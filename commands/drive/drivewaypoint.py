from math import pi

from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import ChassisSpeeds
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import DriverStation, DataLogManager, RobotState, SmartDashboard
from ntcore import NetworkTableInstance

from subsystems.drivesubsystem import DriveSubsystem
from operatorinterface import AnalogInput
import constants
from util.angleoptimize import optimizeAngle


class DriveWaypoint(Command):
    def __init__(
        self, drive: DriveSubsystem, xOffset: AnalogInput, yOffset: AnalogInput
    ) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive

        self.command = Command()

        self.running = False
        self.addRequirements(self.drive)

        self.xoff = xOffset
        self.yoff = yOffset

        self.driveVelocity = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kDriveVelocityKeys, ChassisSpeeds)
            .subscribe(ChassisSpeeds())
        )
        self.waypointAtTarget = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kWaypointAtTargetKey)
            .publish()
        )

        self.xController = ProfiledPIDController(
            constants.kTrajectoryPositionPGainVision,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAcceleration,
            ),
        )
        self.yController = ProfiledPIDController(
            constants.kTrajectoryPositionPGainVision,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAcceleration,
            ),
        )
        self.thetaController = ProfiledPIDControllerRadians(
            constants.kTrajectoryAnglePGain,
            constants.kTrajectoryAngleIGain,
            constants.kTrajectoryAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        SmartDashboard.putData(
            constants.kTargetWaypointXControllerKey, self.xController
        )
        SmartDashboard.putData(
            constants.kTargetWaypointYControllerKey, self.yController
        )
        SmartDashboard.putData(
            constants.kTargetWaypointThetaControllerKey, self.thetaController
        )

        self.thetaController.enableContinuousInput(-pi, pi)

    def initialize(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def execute(self) -> None:
        pass
        # self.command.execute()
        # if self.command.isFinished():
        #     self.command.end(False)
        #     self.running = False

    def isFinished(self) -> bool:
        return False

    def end(self, _interrupted: bool) -> None:
        # pylint: disable=W0212
        AutoBuilder._getPose = self.drive.getPose
        DataLogManager.log("... DONE")


class DriveToReefPosition(DriveWaypoint):
    def __init__(
        self, drive: DriveSubsystem, xOffset: AnalogInput, yOffset: AnalogInput
    ) -> None:
        self.activePub = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kWaypointActiveKey)
            .publish()
        )
        self.targetPub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kTargetWaypointPoseKey, Pose2d)
            .publish()
        )
        DriveWaypoint.__init__(self, drive, xOffset, yOffset)

    def initialize(self):
        self.activePub.set(True)
        self.running = True
        # pylint: disable=W0201
        self.targetPose = self.getClosestPose()
        self.targetPub.set(self.targetPose)

        currentPose = self.drive.getVisionPose()
        self.xController.reset(currentPose.X())
        self.yController.reset(currentPose.Y())

        self.thetaController.reset(self.drive.getRotation().radians(), 0)

        # self.command = AutoBuilder.pathfindToPose(
        #     self.targetPose, constants.kPathfindingConstraints
        # )
        # self.command.initialize()

    def execute(self) -> None:
        currentPose = self.drive.getPose()

        absoluteOutput = ChassisSpeeds(
            self.xController.calculate(
                currentPose.X(),
                self.targetPose.X()
                + self.xoff() * constants.kWaypointJoystickVariation,
            ),
            self.yController.calculate(
                currentPose.Y(),
                self.targetPose.Y()
                + self.yoff() * constants.kWaypointJoystickVariation,
            ),
            self.thetaController.calculate(
                self.drive.getRotation().radians(), self.targetPose.rotation().radians()
            ),
        )

        self.drive.arcadeDriveWithSpeeds(
            absoluteOutput, DriveSubsystem.CoordinateMode.FieldRelative
        )
        self.waypointAtTarget.set(self.atPosition())

    def getClosestPose(self) -> Pose2d:
        currentRotation = self.drive.getRotation()
        targetRotation = Rotation2d.fromDegrees(
            round(currentRotation.degrees() / 60) * 60
        )

        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            for position in (
                constants.kLeftReefToOffsetPositionBlue.values()
                if self.drive.leftReefGetter.get()
                else constants.kRightReefToOffsetPositionBlue.values()
            ):
                if (
                    abs(
                        targetRotation.radians()
                        - optimizeAngle(
                            targetRotation, Rotation2d(position.rotation().Z())
                        ).radians()
                    )
                    <= 0.001
                ):
                    return position.toPose2d()
            return self.drive.getPose()
        else:
            for position in (
                constants.kLeftReefToOffsetPositionRed.values()
                if self.drive.leftReefGetter.get()
                else constants.kRightReefToOffsetPositionRed.values()
            ):
                if (
                    abs(
                        targetRotation.radians()
                        - optimizeAngle(
                            targetRotation, Rotation2d(position.rotation().Z())
                        ).radians()
                    )
                    <= 0.001
                ):
                    return position.toPose2d()
            return self.drive.getPose()


    def atPosition(self) -> bool:
        return (
            self.targetPose.translation().distance(self.drive.getPose().translation())
            < 1 * constants.kMetersPerInch
        )

    def isFinished(self) -> bool:
        return self.atPosition() if RobotState.isAutonomous() else False

    def end(self, _interrupted: bool) -> None:
        # pylint: disable=W0212
        self.running = False
        self.activePub.set(False)
        self.waypointAtTarget.set(False)
        self.drive.arcadeDriveWithSpeeds(
            ChassisSpeeds(), DriveSubsystem.CoordinateMode.FieldRelative
        )
        DataLogManager.log("... DONE")


class SetLeftReef(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        self.setName(__class__.__name__)
        self.drive = drive

    def initialize(self):
        self.drive.leftReefPublisher.set(True)
        self.drive.rightReefPublisher.set(False)

    def isFinished(self):
        return True


class SetRightReef(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        self.setName(__class__.__name__)
        self.drive = drive

    def initialize(self):
        self.drive.leftReefPublisher.set(False)
        self.drive.rightReefPublisher.set(True)

    def isFinished(self):
        return True
