from enum import Enum, auto

from math import sqrt
from typing import Tuple
import typing
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from pathplannerlib.path import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder

from phoenix6.configs.pigeon2_configs import Pigeon2Configuration
from phoenix6.hardware.pigeon2 import Pigeon2
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from wpilib import (
    RobotBase,
    Timer,
    DataLogManager,
    DriverStation,
)

from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Transform2d,
    Translation2d,
    Twist2d,
)
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)
from wpimath.interpolation import TimeInterpolatablePose2dBuffer

import constants
from util import convenientmath
from util.angleoptimize import optimizeAngle
from util.simcoder import CTREEncoder
from util.simtalon import Talon


class SwerveModuleConfigParams:
    swerveEncoderOffset: float
    swerveEncoderID: int
    driveMotorID: int
    driveMotorInverted: bool
    steerMotorID: int
    steerMotorInverted: bool
    canbus: str = ""

    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        driveMotorID: int,
        driveMotorInverted: bool,
        steerMotorID: int,
        steerMotorInverted: bool,
        swerveEncoderID: int,
        swerveEncoderOffset: float,
        canbus: str = "",
    ) -> None:
        self.driveMotorID = driveMotorID
        self.driveMotorInverted = driveMotorInverted
        self.steerMotorID = steerMotorID
        self.steerMotorInverted = steerMotorInverted
        self.swerveEncoderID = swerveEncoderID
        self.swerveEncoderOffset = swerveEncoderOffset
        self.canbus = canbus


class SwerveModule:
    def __init__(self, name: str) -> None:
        self.name = name

    def getSwerveAngle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelLinearVelocity(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelTotalPosition(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def reset(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def optimizedAngle(self, targetAngle: Rotation2d) -> Rotation2d:
        return optimizeAngle(self.getSwerveAngle(), targetAngle)

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.getWheelTotalPosition(), self.getSwerveAngle())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.getWheelLinearVelocity(),
            self.getSwerveAngle(),
        )

    def applyState(self, state: SwerveModuleState) -> None:
        state.optimize(self.getSwerveAngle())

        self.setWheelLinearVelocityTarget(state.speed)
        if (
            abs(state.speed) >= constants.kMinWheelLinearVelocity
        ):  # prevent unneccisary movement for what would otherwise not move the robot
            optimizedAngle = self.optimizedAngle(state.angle)
            self.setSwerveAngleTarget(optimizedAngle)


class CTRESwerveModule(SwerveModule):
    """
    Implementation of SwerveModule for the SDS swerve modules
    https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module
        driveMotor: Kraken X60 Motor (with built-in encoder) attached to wheel through gearing
        steerMotor: Falcon 500 Motor (with built-in encoder) attached to swerve through gearing
        swerveEncoder: CANCoder
    """

    def __init__(self, name: str, config: SwerveModuleConfigParams) -> None:
        SwerveModule.__init__(self, name)
        DataLogManager.log(f"Initializing swerve module: {self.name}")
        DataLogManager.log(f"   Configuring drive motor: CAN ID: {config.driveMotorID}")
        self.driveMotor = Talon(
            config.driveMotorID,
            f"Drive Motor {name}",
            constants.kDrivePGain,
            constants.kDriveIGain,
            constants.kDriveDGain,
            config.driveMotorInverted,
            config.canbus,
            constants.kDriveVGain,
        )
        self.driveMotor.setNeutralMode(Talon.NeutralMode.Brake)
        if RobotBase.isReal():
            self.driveMotor.setCurrentLimit(constants.kDriveCurrentLimit)
        DataLogManager.log("   ... Done")
        DataLogManager.log(f"   Configuring steer motor: CAN ID: {config.steerMotorID}")
        self.steerMotor = Talon(
            config.steerMotorID,
            f"Steer Motor {name}",
            constants.kSteerPGain,
            constants.kSteerIGain,
            constants.kSteerDGain,
            config.steerMotorInverted,
            config.canbus,
        )
        DataLogManager.log("   ... Done")
        DataLogManager.log(
            f"   Configuring swerve encoder: CAN ID: {config.swerveEncoderID}"
        )
        self.swerveEncoder = CTREEncoder(
            config.swerveEncoderID, config.swerveEncoderOffset, config.canbus
        )
        DataLogManager.log("   ... Done")
        DataLogManager.log("... Done")

    def getSwerveAngle(self) -> Rotation2d:
        steerRotation = self.steerMotor.get(Talon.ControlMode.Position)
        swerveAngle = (
            steerRotation
            / constants.kSteerGearingRatio
            * constants.kRadiansPerRevolution
        )
        return Rotation2d(swerveAngle)

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        steerEncoderPulses = (
            (swerveAngle.radians())
            / constants.kRadiansPerRevolution
            * constants.kSteerGearingRatio
        )
        self.steerMotor.setEncoderPosition(steerEncoderPulses)

    def getSwerveEncoderAngle(self) -> Rotation2d:
        return self.swerveEncoder.getPosition()

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        steerEncoderPulsesTarget = (
            swerveAngleTarget.radians()
            / constants.kRadiansPerRevolution
            * constants.kSteerGearingRatio
        )
        self.steerMotor.set(Talon.ControlMode.Position, steerEncoderPulsesTarget)

    def getWheelLinearVelocity(self) -> float:
        driveEncoderPulsesPerSecond = self.driveMotor.get(Talon.ControlMode.Velocity)
        wheelLinearVelocity = (
            driveEncoderPulsesPerSecond
            * constants.kWheelRadius
            * constants.kRadiansPerRevolution
            / constants.kDriveGearingRatio
        )
        return wheelLinearVelocity

    def getWheelTotalPosition(self) -> float:
        driveEncoderPulses = self.driveMotor.get(Talon.ControlMode.Position)
        driveDistance = (
            driveEncoderPulses
            * constants.kWheelRadius
            * constants.kRadiansPerRevolution
            / constants.kDriveGearingRatio
        )
        return driveDistance

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        driveEncoderPulsesPerSecond = (
            wheelLinearVelocityTarget
            / constants.kWheelRadius
            / constants.kRadiansPerRevolution
            * constants.kDriveGearingRatio
        )
        self.driveMotor.set(
            Talon.ControlMode.Velocity,
            driveEncoderPulsesPerSecond,
        )

    def reset(self) -> None:
        if RobotBase.isReal():
            self.setSwerveAngle(self.swerveEncoder.getPosition())

    def getSimulator(
        self,
    ) -> tuple[
        typing.Callable[[], TalonFXSimState],
        typing.Callable[[], TalonFXSimState],
        typing.Callable[[], CANcoderSimState],
    ]:
        return (
            self.driveMotor.getSimCollection,
            self.steerMotor.getSimCollection,
            self.swerveEncoder.getSim,
        )


# pylint: disable-next=too-many-instance-attributes
class DriveSubsystem(Subsystem):
    class CoordinateMode(Enum):
        RobotRelative = auto()
        FieldRelative = auto()
        TargetRelative = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.rotationOffset = 0

        self.frontLeftModule = CTRESwerveModule(
            constants.kFrontLeftModuleName,
            SwerveModuleConfigParams(
                constants.kFrontLeftDriveMotorId,
                constants.kFrontLeftDriveInverted,
                constants.kFrontLeftSteerMotorId,
                constants.kFrontLeftSteerInverted,
                constants.kFrontLeftSteerEncoderId,
                constants.kFrontLeftAbsoluteEncoderOffset,
                constants.kCANivoreName,
            ),
        )
        self.frontRightModule = CTRESwerveModule(
            constants.kFrontRightModuleName,
            SwerveModuleConfigParams(
                constants.kFrontRightDriveMotorId,
                constants.kFrontRightDriveInverted,
                constants.kFrontRightSteerMotorId,
                constants.kFrontRightSteerInverted,
                constants.kFrontRightSteerEncoderId,
                constants.kFrontRightAbsoluteEncoderOffset,
                constants.kCANivoreName,
            ),
        )
        self.backLeftModule = CTRESwerveModule(
            constants.kBackLeftModuleName,
            SwerveModuleConfigParams(
                constants.kBackLeftDriveMotorId,
                constants.kBackLeftDriveInverted,
                constants.kBackLeftSteerMotorId,
                constants.kBackLeftSteerInverted,
                constants.kBackLeftSteerEncoderId,
                constants.kBackLeftAbsoluteEncoderOffset,
                constants.kCANivoreName,
            ),
        )
        self.backRightModule = CTRESwerveModule(
            constants.kBackRightModuleName,
            SwerveModuleConfigParams(
                constants.kBackRightDriveMotorId,
                constants.kBackRightDriveInverted,
                constants.kBackRightSteerMotorId,
                constants.kBackRightSteerInverted,
                constants.kBackRightSteerEncoderId,
                constants.kBackRightAbsoluteEncoderOffset,
                constants.kCANivoreName,
            ),
        )

        self.modules = (
            self.frontLeftModule,
            self.frontRightModule,
            self.backLeftModule,
            self.backRightModule,
        )

        self.kinematics = SwerveDrive4Kinematics(
            constants.kFrontLeftWheelPosition,
            constants.kFrontRightWheelPosition,
            constants.kBackLeftWheelPosition,
            constants.kBackRightWheelPosition,
        )

        # Create the gyro, a sensor which can indicate the heading of the robot relative
        # to a customizable position.
        self.gyro = Pigeon2(constants.kPigeonCANId, constants.kCANivoreName)

        toApply = Pigeon2Configuration()
        self.gyro.configurator.apply(toApply)
        self.gyro.get_yaw().set_update_frequency(100)

        self.estimator = RobotPoseEstimator(
            self.kinematics,
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            Pose2d(),
            (0.05, 0.05, 5 * constants.kRadiansPerDegree),
        )
        # standard deviations stolen from 2910

        # Create the an object for our odometry, which will utilize sensor data to
        # keep a record of our position on the field.
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            Pose2d(),
        )
        self.printTimer = Timer()
        self.printTimer.start()
        self.vxLimiter = SlewRateLimiter(constants.kDriveAccelLimit)
        self.vyLimiter = SlewRateLimiter(constants.kDriveAccelLimit)

        self.visionEstimate = Pose2d()

        self.pastTime = self.printTimer.get()

        self.config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose,
            self.resetDriveAtPosition,
            self.getRobotRelativeSpeeds,
            self.drivePathPlanned,
            PPHolonomicDriveController(
                constants.kPathFollowingTranslationConstantsAuto,
                constants.kPathFollowingRotationConstants,
            ),
            # controller
            self.config,
            # robot_config
            (lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed),
            self,
        )

        self.swerveStatePublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic(constants.kSwerveActualStatesKey, SwerveModuleState)
            .publish()
        )
        self.swerveStateExpectedPublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic(constants.kSwerveExpectedStatesKey, SwerveModuleState)
            .publish()
        )
        self.robotPosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotPoseArrayKeys.valueKey, Pose2d)
            .publish()
        )
        self.robotPoseValidPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kRobotPoseArrayKeys.validKey)
            .publish()
        )
        self.robotPoseValidPublisher.set(True)

        self.leftReefPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kWaypointLeftReefKey)
            .publish()
        )
        self.leftReefPublisher.set(True)
        self.leftReefGetter = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kWaypointLeftReefKey)
            .subscribe(True)
        )

        self.rightReefPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kWaypointRightReefKey)
            .publish()
        )
        self.rightReefPublisher.set(False)
        # self.visionPosePublisher = (
        #     NetworkTableInstance.getDefault()
        #     .getStructTopic(constants.kRobotVisionPoseArrayKeys.valueKey, Pose2d)
        #     .publish()
        # )

        # self.visionPoseValidPublisher = (
        #     NetworkTableInstance.getDefault()
        #     .getBooleanTopic(constants.kRobotVisionPoseArrayKeys.validKey)
        #     .publish()
        # )

        self.driveVelocityPublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kDriveVelocityKeys, ChassisSpeeds)
            .publish()
        )
        self.targetAngleRobotRelative = (
            NetworkTableInstance.getDefault()
            .getStructTopic(
                constants.kTargetAngleRelativeToRobotKeys.valueKey, Rotation2d
            )
            .subscribe(Rotation2d())
        )
        self.targetAngleRobotRelativeValid = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kTargetAngleRelativeToRobotKeys.valueKey)
            .subscribe(False)
        )

        if RobotBase.isSimulation():
            self.simVelocityGetter = (
                NetworkTableInstance.getDefault()
                .getStructTopic(constants.kSimRobotVelocityArrayKey, ChassisSpeeds)
                .subscribe(ChassisSpeeds())
            )

        self.useVisionPose = False
        self.visionPoseGetter = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotVisionPose1ArrayKeys.valueKey, Pose2d)
            .subscribe(Pose2d())
        )

    def defenseState(self):
        def setModuleTo(module: SwerveModule, angle: Rotation2d):
            module.setWheelLinearVelocityTarget(0)
            module.setSwerveAngleTarget(module.optimizedAngle(angle))

        setModuleTo(self.frontLeftModule, Rotation2d.fromDegrees(45))
        setModuleTo(self.frontRightModule, Rotation2d.fromDegrees(-45))
        setModuleTo(self.backLeftModule, Rotation2d.fromDegrees(135))
        setModuleTo(self.backRightModule, Rotation2d.fromDegrees(-135))

    def resetDriveAtPosition(self, pose: Pose2d):
        self.resetSwerveModules()
        self.resetGyro(pose)

    def getRobotRelativeSpeeds(self):
        return self.kinematics.toChassisSpeeds(self.getModuleStates())

    def getModuleStates(self):
        return (
            self.frontLeftModule.getState(),
            self.frontRightModule.getState(),
            self.backLeftModule.getState(),
            self.backRightModule.getState(),
        )

    def resetSwerveModules(self):
        for module in self.modules:
            module.reset()
        self.resetGyro(Pose2d())

    def setOdometryPosition(self, pose: Pose2d):
        # self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.rotationOffset = pose.rotation().degrees()
        self.resetOdometryAtPosition(pose)

    def resetGyro(self, pose: Pose2d):
        self.gyro.set_yaw(0)
        # self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.rotationOffset = pose.rotation().degrees()
        self.resetOdometryAtPosition(pose)

        # if RobotBase.isSimulation():
        #     self.resetSimPosition(pose)

    def getVisionPose(self) -> Pose2d:
        return self.visionPoseGetter.get()

    def getPose(self) -> Pose2d:
        if self.useVisionPose:
            return self.visionPoseGetter.get()
        translation = self.estimator.estimatedPose.translation()
        rotation = self.getRotation()
        return Pose2d(translation, rotation)

    def applyStates(
        self,
        moduleStates: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        (
            frontLeftState,
            frontRightState,
            backLeftState,
            backRightState,
        ) = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates, constants.kMaxWheelLinearVelocity
        )

        self.swerveStateExpectedPublisher.set(
            [frontLeftState, frontRightState, backLeftState, backRightState]
        )
        self.frontLeftModule.applyState(frontLeftState)
        self.frontRightModule.applyState(frontRightState)
        self.backLeftModule.applyState(backLeftState)
        self.backRightModule.applyState(backRightState)

    def getRotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value + self.rotationOffset)

    def getAngularVelocity(self) -> float:
        """radians"""
        if RobotBase.isSimulation():
            value: ChassisSpeeds = self.simVelocityGetter.get()
            return value.omega
        return (
            self.gyro.get_angular_velocity_z_world().value * constants.kRadiansPerDegree
        )

    def getPitch(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.gyro.get_pitch().value + 180)

    def resetOdometryAtPosition(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            pose,
        )
        self.estimator.resetPosition(
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            pose,
        )

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's
        odometry with sensor data.
        """

        swervePositions = (
            self.frontLeftModule.getPosition(),
            self.frontRightModule.getPosition(),
            self.backLeftModule.getPosition(),
            self.backRightModule.getPosition(),
        )
        self.odometry.update(self.getRotation(), swervePositions)
        robotPose = self.getPose()

        self.swerveStatePublisher.set(
            [
                self.frontLeftModule.getState(),
                self.frontRightModule.getState(),
                self.backLeftModule.getState(),
                self.backRightModule.getState(),
            ]
        )

        # robotPoseArray = [robotPose.X(), robotPose.Y(), robotPose.rotation().radians()]

        self.robotPosePublisher.set(robotPose)
        self.robotPoseValidPublisher.set(True)

        odoMeasure = OdometryObservation(
            [*swervePositions], self.getRotation(), self.printTimer.getFPGATimestamp()
        )
        self.estimator.addOdometryMeasurement(odoMeasure)
        self.visionEstimate = self.estimator.estimatedPose
        # self.visionPosePublisher.set(self.visionEstimate)

        # curTime = self.printTimer.get()
        # if self.printTimer.hasElapsed(constants.kPrintPeriod):
        #     # DataLogManager.log(
        #     #     # pylint:disable-next=consider-using-f-string
        #     #     "r: {:.1f}, {:.1f}, {:.0f}* fl: {:.0f}* {:.1f} fr: {:.0f}* {:.1f} bl: {:.0f}* {:.1f} br: {:.0f}* {:.1f}".format(
        #     #         robotPose.X(),
        #     #         robotPose.Y(),
        #     #         robotPose.rotation().degrees(),
        #     #         self.frontLeftModule.getSwerveAngle().degrees(),
        #     #         self.frontLeftModule.getWheelLinearVelocity(),
        #     #         self.frontRightModule.getSwerveAngle().degrees(),
        #     #         self.frontRightModule.getWheelLinearVelocity(),
        #     #         self.backLeftModule.getSwerveAngle().degrees(),
        #     #         self.backLeftModule.getWheelLinearVelocity(),
        #     #         self.backRightModule.getSwerveAngle().degrees(),
        #     #         self.backRightModule.getWheelLinearVelocity(),
        #     #     )
        #     # )
        #     DataLogManager.log(
        #         f"Timing: {1 / (curTime - self.pastTime)}, {delta1}, {delta2}, {delta3}, {delta4}, {delta5}"
        #     )
        # self.pastTime = curTime

    def arcadeDriveWithFactors(
        self,
        forwardSpeedFactor: float,
        sidewaysSpeedFactor: float,
        rotationSpeedFactor: float,
        coordinateMode: CoordinateMode,
    ) -> None:
        """
        Drives the robot using arcade controls.

        :param forwardSpeedFactor: the commanded forward movement
        :param sidewaysSpeedFactor: the commanded sideways movement
        :param rotationSpeedFactor: the commanded rotation
        """

        forwardSpeedFactor = convenientmath.clamp(forwardSpeedFactor, -1, 1)
        sidewaysSpeedFactor = convenientmath.clamp(sidewaysSpeedFactor, -1, 1)
        rotationSpeedFactor = convenientmath.clamp(rotationSpeedFactor, -1, 1)

        combinedLinearFactor = Translation2d(
            forwardSpeedFactor, sidewaysSpeedFactor
        ).norm()

        # prevent combined forward & sideways inputs from exceeding the max linear velocity
        if combinedLinearFactor > 1.0:
            forwardSpeedFactor = forwardSpeedFactor / combinedLinearFactor
            sidewaysSpeedFactor = sidewaysSpeedFactor / combinedLinearFactor

        chassisSpeeds = ChassisSpeeds(
            forwardSpeedFactor * constants.kMaxForwardLinearVelocity,
            sidewaysSpeedFactor * constants.kMaxSidewaysLinearVelocity,
            rotationSpeedFactor * constants.kMaxRotationAngularVelocity,
        )

        self.arcadeDriveWithSpeeds(chassisSpeeds, coordinateMode)

    def drivePathPlanned(self, chassisSpeeds: ChassisSpeeds, _feedForward):
        return self.arcadeDriveWithSpeeds(
            chassisSpeeds, DriveSubsystem.CoordinateMode.RobotRelative
        )

    def arcadeDriveWithSpeeds(
        self, chassisSpeeds: ChassisSpeeds, coordinateMode: CoordinateMode
    ) -> None:
        targetAngle = self.targetAngleRobotRelative.get()
        discritizedSpeeds = ChassisSpeeds.discretize(
            chassisSpeeds, constants.kRobotUpdatePeriod
        )

        robotChassisSpeeds = None
        if coordinateMode is DriveSubsystem.CoordinateMode.RobotRelative:
            robotChassisSpeeds = discritizedSpeeds
        elif coordinateMode is DriveSubsystem.CoordinateMode.FieldRelative:
            robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                discritizedSpeeds.vx,
                discritizedSpeeds.vy,
                discritizedSpeeds.omega,
                self.getRotation()
                + Rotation2d(
                    self.getAngularVelocity() * constants.kDriveAngularVelocityCoeff
                ),
            )
        elif coordinateMode is DriveSubsystem.CoordinateMode.TargetRelative:
            if self.targetAngleRobotRelativeValid.get():
                robotSpeeds = Translation2d(chassisSpeeds.vx, chassisSpeeds.vy)
                targetAlignedSpeeds = robotSpeeds.rotateBy(targetAngle)
                robotChassisSpeeds = ChassisSpeeds(
                    targetAlignedSpeeds.X(),
                    targetAlignedSpeeds.Y(),
                    chassisSpeeds.omega,
                )
            else:
                robotChassisSpeeds = ChassisSpeeds()

        fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            robotChassisSpeeds.vx,
            robotChassisSpeeds.vy,
            robotChassisSpeeds.omega,
            -self.getRotation(),
        )

        self.driveVelocityPublisher.set(fieldSpeeds)

        moduleStates = self.kinematics.toSwerveModuleStates(robotChassisSpeeds)
        self.applyStates(moduleStates)


# shamelessly reimplemented from 6328
class OdometryObservation:
    def __init__(
        self,
        wheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        gyroAngle: Rotation2d,
        timestamp: float,
    ) -> None:
        self.wheelPositions = wheelPositions
        self.gyroAngle = gyroAngle
        self.timestamp = timestamp


class VisionObservation:
    def __init__(self, visionPose: Pose2d, timestamp: float, std: list[float]) -> None:
        assert len(std) == 3
        self.visionPose = visionPose
        self.timestamp = timestamp
        self.std = std


class RobotPoseEstimator:
    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        kinematics: SwerveDrive4Kinematics,
        gyro: Rotation2d,
        startWheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        startPose: Pose2d,
        odoStdDevs: tuple[float, float, float],
    ) -> None:
        self.kinematics = kinematics
        self.lastGyroAngle = gyro
        self.lastWheelPositions = startWheelPositions
        self.odometryPose = startPose
        self.estimatedPose = startPose

        self.odoStdDevs = odoStdDevs

        self.poseBuffer = TimeInterpolatablePose2dBuffer(2.0)

    def addOdometryMeasurement(self, measurement: OdometryObservation):
        newPositions: list[SwerveModulePosition] = []
        for old, new in zip(self.lastWheelPositions, measurement.wheelPositions):
            newPositions.append(
                SwerveModulePosition(new.distance - old.distance, new.angle)
            )

        twist = self.kinematics.toTwist2d(newPositions)
        twist = Twist2d(
            twist.dx, twist.dy, (measurement.gyroAngle - self.lastGyroAngle).radians()
        )

        self.odometryPose = self.odometryPose.exp(twist)
        self.poseBuffer.addSample(measurement.timestamp, self.odometryPose)

        self.estimatedPose = self.estimatedPose.exp(twist)

        self.lastGyroAngle = measurement.gyroAngle
        self.lastWheelPositions = measurement.wheelPositions

    def addVisionMeasurement(self, measurement: VisionObservation):
        # check if measurement is valid enough to work with
        if self.poseBuffer.getInternalBuffer()[-1][0] - 2.0 > measurement.timestamp:
            return

        sample = self.poseBuffer.sample(measurement.timestamp)
        if sample is None:
            return

        sampleToOdometryTransform = Transform2d(self.odometryPose, sample)
        odometryToSampleTransform = Transform2d(sample, self.odometryPose)

        estimateAtTime = self.estimatedPose + odometryToSampleTransform

        # new vision matrix
        r = [i * i for i in measurement.std]

        # Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        # and C = I. See wpimath/algorithms.md.
        visionK = [0.0, 0.0, 0.0]

        for i in range(3):
            stdDev = self.odoStdDevs[i]
            if stdDev == 0.0:
                visionK[i] = 0.0
            else:
                visionK[i] = stdDev / (stdDev + sqrt(stdDev * r[i]))

        transform = Transform2d(estimateAtTime, measurement.visionPose)
        kTimesTransform = [
            visionK[i] * k
            for i, k in enumerate(
                [transform.X(), transform.Y(), transform.rotation().radians()]
            )
        ]

        scaledTransform = Transform2d(
            kTimesTransform[0], kTimesTransform[1], kTimesTransform[2]
        )

        self.estimatedPose = (
            estimateAtTime + scaledTransform + sampleToOdometryTransform
        )

    def resetPosition(
        self,
        gyro: Rotation2d,
        startWheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        startPose: Pose2d,
    ):
        self.lastGyroAngle = gyro
        self.estimatedPose = startPose
        self.odometryPose = startPose
        self.lastWheelPositions = startWheelPositions
