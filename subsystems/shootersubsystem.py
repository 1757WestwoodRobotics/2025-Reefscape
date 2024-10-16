import math
from commands2 import Subsystem
from wpilib import SmartDashboard, Timer, RobotBase
from wpilib._wpilib import DriverStation
from wpimath.controller import SimpleMotorFeedforwardMeters, ArmFeedforward
from wpimath.geometry import Rotation2d, Pose3d, Pose2d, Rotation3d
from phoenix6.configs import CurrentLimitsConfigs
from util.simtalon import Talon
from util.simcoder import CTREEncoder
from util.advantagescopeconvert import convertToSendablePoses
from util.convenientmath import clamp
from util.getsdarray import getSDArray, putSDArray
import constants


class SimNote:
    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
        initialTime: float,
    ) -> None:
        # metric units
        # initial position
        self.xi = x
        self.yi = y
        self.zi = z

        # current position
        self.xc = x
        self.yc = y
        self.zc = z

        # velocity
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.initialTime = initialTime

    def update(self, currentTime) -> None:
        if self.zc > 0:
            elapsedTime = currentTime - self.initialTime
            self.xc = self.xi + self.vx * elapsedTime
            self.yc = self.yi + self.vy * elapsedTime
            self.zc = (
                self.zi
                + self.vz * elapsedTime
                - constants.kGravity / 2 * elapsedTime**2
            )


class ShooterSubsystem(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.timer = Timer()
        self.simNotes = []

        self.angleMotor = Talon(
            constants.kAngleMotorCANId,
            constants.kAngleMotorName,
            constants.kAngleMotorPGain,
            constants.kAngleMotorIGain,
            constants.kAngleMotorDGain,
            constants.kAngleMotorInverted,
            moMagicAccel=constants.kAngleMotorAccel,
            moMagicVel=constants.kAngleMotorVel,
        )
        self.angleMotor.setNeutralMode(Talon.NeutralMode.Brake)
        self.leftShootingMotor = Talon(
            constants.kLeftShootingMotorCANId,
            constants.kLeftShootingMotorName,
            constants.kLeftShootingMotorPGain,
            constants.kLeftShootingMotorIGain,
            constants.kLeftShootingMotorDGain,
            constants.kLeftShootingMotorInverted,
            kV=constants.kLeftShootingMotorKv,
        )
        self.rightShootingMotor = Talon(
            constants.kRightShootingMotorCANId,
            constants.kRightShootingMotorName,
            constants.kRightShootingMotorPGain,
            constants.kRightShootingMotorIGain,
            constants.kRightShootingMotorDGain,
            constants.kRightShootingMotorInverted,
            kV=constants.kLeftShootingMotorKv,
        )

        self.shooterEncoder = CTREEncoder(
            constants.kShooterAngleEncoderCANId, constants.kShooterAngleEncoderOffset
        )
        self.shooterInitPosition = self.shooterEncoder.getPosition()

        self.leftShootingMotor.setCurrentLimit(
            CurrentLimitsConfigs().with_supply_current_limit(
                constants.kShootingMotorCurrentLimit
            )
        )
        self.rightShootingMotor.setCurrentLimit(
            CurrentLimitsConfigs().with_supply_current_limit(
                constants.kShootingMotorCurrentLimit
            )
        )

        self.angleMotor.setCurrentLimit(
            CurrentLimitsConfigs().with_supply_current_limit(
                constants.kAngleMotorCurrentLimit
            )
        )

        # set motor position, encoder is inverse of the shooter

        self.angleMotor.setEncoderPosition(
            self.shooterEncoder.getPosition().radians()
            / constants.kRadiansPerRevolution
            * constants.kAngleMotorRatio
        )

        self.targetAngle = Rotation2d()
        self.leftTargetSpeed = 0
        self.rightTargetSpeed = 0

        self.ff = SimpleMotorFeedforwardMeters(
            constants.kLeftShootingMotorKs, constants.kLeftShootingMotorKv
        )
        self.pivotff = ArmFeedforward(0, 0.13, 0)

        SmartDashboard.putNumber(constants.kLeftMotorFudgeKey, 0)
        SmartDashboard.putNumber(constants.kRightMotorFudgeKey, 0)
        SmartDashboard.putNumber(constants.kShooterAngleFudgeKey, 0)
        # speeds are in RPM, same as velocity control mode

    def resetShooterPivot(self):
        SmartDashboard.putNumber(constants.kShooterAngleFudgeKey, 0)
        self.angleMotor.setEncoderPosition(
            self.shooterEncoder.getPosition().radians()
            / constants.kRadiansPerRevolution
            * constants.kAngleMotorRatio
            * -1
        )

    def setShooterAngle(self, angle: Rotation2d) -> None:
        self.targetAngle = (
            Rotation2d(
                clamp(
                    angle.radians(),
                    constants.kShooterMinAngle.radians(),
                    constants.kShooterMaxAngle.radians(),
                )
            )
            + Rotation2d(SmartDashboard.getNumber(constants.kShooterAngleFudgeKey, 0))
            + (
                constants.kShooterFudgeGlobalRed
                if DriverStation.getAlliance() == DriverStation.Alliance.kRed
                else constants.kShooterFudgeGlobalBlue
            )
        )

        if (
            SmartDashboard.getBoolean(constants.kIntakeAtPositionKey, False)
            and SmartDashboard.getString(constants.kIntakeStateKey, "Intaking")
            != "Intaking"
        ):
            self.angleMotor.set(
                Talon.ControlMode.MotionMagic,
                self.targetAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kAngleMotorRatio,
                self.pivotff.calculate(self.targetAngle.radians(), 0),
            )
        else:
            self.safePivot()

    def setLeftShootingMotorSpeed(self, rpm: float) -> None:
        self.leftTargetSpeed = (
            (rpm + SmartDashboard.getNumber(constants.kLeftMotorFudgeKey, 0))
            * constants.kShootingMotorRatio
            / 60
        )
        self.leftShootingMotor.set(
            Talon.ControlMode.Velocity,
            self.leftTargetSpeed,
            self.ff.calculate(self.leftTargetSpeed),
        )

    def setRightShootingMotorSpeed(self, rpm: float) -> None:
        self.rightTargetSpeed = (
            (rpm + SmartDashboard.getNumber(constants.kRightMotorFudgeKey, 0))
            * constants.kShootingMotorRatio
            / 60
        )
        self.rightShootingMotor.set(
            Talon.ControlMode.Velocity,
            self.rightTargetSpeed,
            self.ff.calculate(self.rightTargetSpeed),
        )

    def neutralShooter(self) -> None:
        if not DriverStation.isAutonomous():
            self.rightShootingMotor.neutralOutput()
            self.leftShootingMotor.neutralOutput()
        # self.setLeftShootingMotorSpeed(
        #     constants.kShooterIdleSpeed
        # )
        # self.setRightShootingMotorSpeed(
        #     constants.kShooterIdleSpeed
        # )
        self.safePivot()

    def safePivot(self) -> None:
        self.targetAngle = self.shooterInitPosition
        self.angleMotor.set(  # in neutral ignore the fudge
            Talon.ControlMode.MotionMagic,
            self.shooterInitPosition.radians()
            / constants.kRadiansPerRevolution
            * constants.kAngleMotorRatio,
        )

    def getShooterAngle(self) -> Rotation2d:
        return Rotation2d(
            self.angleMotor.get(Talon.ControlMode.Position)
            * constants.kRadiansPerRevolution
            / constants.kAngleMotorRatio
        )

    def getShooterAngleAbsolute(self) -> Rotation2d:
        return self.shooterEncoder.getPosition()

    def getLeftShooterSpeed(self) -> int:
        # RPM
        return self.leftShootingMotor.get(Talon.ControlMode.Velocity)

    def getRightShooterSpeed(self) -> int:
        # RPM
        return self.rightShootingMotor.get(Talon.ControlMode.Velocity)

    def angleOnTarget(self) -> bool:
        return (
            abs(
                self.targetAngle.radians()
                - (
                    self.getShooterAngleAbsolute().radians()
                    if RobotBase.isReal()
                    else self.getShooterAngle().radians()
                )
            )
            < constants.kShooterAngleTolerance.radians()
        )

    def leftMotorSpeedOnTarget(self) -> bool:
        return (
            abs(self.leftTargetSpeed - self.getLeftShooterSpeed())
            < constants.kShooterSpeedTolerance
        ) or self.getLeftShooterSpeed() > 65

    def rightMotorSpeedOnTarget(self) -> bool:
        return (
            abs(self.rightTargetSpeed - self.getRightShooterSpeed())
            < constants.kShooterSpeedTolerance
        ) or self.getRightShooterSpeed() > 65

    def readyToShoot(self) -> bool:
        return (
            self.angleOnTarget()
            and self.leftMotorSpeedOnTarget()
            and self.rightMotorSpeedOnTarget()
            and self.angleOnTarget()
        )

    def addSimNote(self) -> None:
        pose = getSDArray(constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0])
        robotPose = Pose2d(*pose)
        robotVelocities = getSDArray(constants.kDriveVelocityKeys, [0, 0, 0])

        shooterPose = Pose3d(robotPose) + constants.kRobotToShooterTransform

        noteSpeed = (
            (
                SmartDashboard.getNumber(constants.kRightShootingMotorSpeedKey, 0)
                + SmartDashboard.getNumber(constants.kLeftShootingMotorSpeedKey, 0)
            )
            / 2
            / constants.kSecondsPerMinute
            * constants.kRadiansPerRevolution
            * constants.kShooterWheelDiameter
            / 2
        )
        shooterAngle = SmartDashboard.getNumber(constants.kShooterAngleKey, 0)
        vVertical = noteSpeed * math.sin(shooterAngle)
        vHorizontal = noteSpeed * math.cos(shooterAngle)

        vx = robotVelocities[0] + vHorizontal * math.cos(shooterPose.rotation().Z())
        vy = robotVelocities[1] + vHorizontal * math.sin(shooterPose.rotation().Z())
        vz = vVertical

        self.simNotes.append(
            SimNote(
                shooterPose.X(),
                shooterPose.Y(),
                shooterPose.Z(),
                vx,
                vy,
                vz,
                self.timer.getFPGATimestamp(),
            )
        )
        latestNoteTrajectory = []
        onGround = False
        elapsedTime = 0
        while not onGround:
            note = SimNote(
                shooterPose.X(),
                shooterPose.Y(),
                shooterPose.Z(),
                vx,
                vy,
                vz,
                self.timer.getFPGATimestamp(),
            )
            note.update(self.timer.getFPGATimestamp() + elapsedTime)
            elapsedTime += constants.kNoteTrajectoryTimeInterval
            if note.zc > 0:
                latestNoteTrajectory.append(
                    Pose3d(note.xc, note.yc, note.zc, Rotation3d(0, 0, 0))
                )
            else:
                onGround = True
        putSDArray(
            constants.kLatestNoteTrajectoryKey,
            convertToSendablePoses(latestNoteTrajectory),
        )

    def periodic(self) -> None:
        notePoses = []

        for simNote in self.simNotes:
            simNote.update(self.timer.getFPGATimestamp())
            if simNote.zc <= 0:
                self.simNotes.remove(simNote)
            else:
                notePoses.append(
                    Pose3d(simNote.xc, simNote.yc, simNote.zc, Rotation3d(0, 0, 0))
                )
        putSDArray(constants.kSimNoteArrayKey, convertToSendablePoses(notePoses))
        # logging
        if not SmartDashboard.getBoolean(constants.kShooterManualModeKey, False):
            SmartDashboard.putNumber(
                constants.kShooterAngleKey,
                (
                    self.getShooterAngleAbsolute().radians()
                    if RobotBase.isReal()
                    else self.getShooterAngle().radians()
                ),
            )
            SmartDashboard.putNumber(
                constants.kShooterAngleMotorKey, self.getShooterAngle().radians()
            )
            SmartDashboard.putNumber(
                constants.kLeftShootingMotorSpeedKey, self.getLeftShooterSpeed()
            )
            SmartDashboard.putNumber(
                constants.kRightShootingMotorSpeedKey, self.getRightShooterSpeed()
            )

        SmartDashboard.putNumber(
            constants.kShooterPivotTargetKey, self.targetAngle.radians()
        )
        SmartDashboard.putNumber(
            constants.kLeftShootingMotorTargetKey, self.leftTargetSpeed
        )
        SmartDashboard.putNumber(
            constants.kRightShootingMotorTargetKey, self.rightTargetSpeed
        )

        SmartDashboard.putBoolean(
            constants.kShooterAngleOnTargetKey, self.angleOnTarget()
        )
        SmartDashboard.putBoolean(
            constants.kLeftShootingMotorOnTargetKey, self.leftMotorSpeedOnTarget()
        )
        SmartDashboard.putBoolean(
            constants.kRightShootingMotorOnTargetKey, self.rightMotorSpeedOnTarget()
        )
        SmartDashboard.putBoolean(constants.kReadyToShoot, self.readyToShoot())
