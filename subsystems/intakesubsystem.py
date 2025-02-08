from enum import Enum, auto
from commands2 import Subsystem
from wpilib import RobotBase
from wpimath.geometry import Rotation2d
from ntcore import NetworkTableInstance
from util.simtalon import Talon
from util.simcoder import CTREEncoder
from util.angleoptimize import intakeAccountForSillyEncoder
from util.convenientmath import clamp
import constants


class IntakeSubsystem(Subsystem):
    class IntakeState(Enum):
        Intaking = auto()
        Idle = auto()
        Scoring = auto()
        Knock = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)  # basic subsystem boilerplate

        self.pivotEncoder = CTREEncoder(
            constants.kPivotEncoderID, constants.kPivotEncoderOffset
        )

        self.intakeMotor = Talon(
            constants.kIntakeCANID,
            constants.kIntakeName,
            constants.kIntakePGain,
            constants.kIntakeIGain,
            constants.kIntakeDGain,
            constants.kIntakeInverted,
        )
        self.intakeMotor.setCurrentLimit(constants.kIntakeCurrentLimit)
        self.pivotMotor = Talon(
            constants.kPivotCANID,
            constants.kPivotName,
            constants.kPivotPGain,
            constants.kPivotIGain,
            constants.kPivotDGain,
            constants.kPivotInverted,
            moMagicAccel=constants.kPivotAccel,
            moMagicVel=constants.kPivotVel,
        )
        self.pivotMotor.setNeutralMode(Talon.NeutralMode.Brake)
        # self.positionDebouncer = Debouncer(0.1, Debouncer.DebounceType.kRising)

        self.resetPivot()

        self.state = self.IntakeState.Idle
        self.targetAngle = Rotation2d()

        self.intakeAtPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kIntakeAtPositionKey)
            .publish()
        )
        self.pivotAnglePublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kPivotAngleKey)
            .publish()
        )
        self.intakeStatePublisher = (
            NetworkTableInstance.getDefault()
            .getStringTopic(constants.kIntakeStateKey)
            .publish()
        )

    def periodic(self) -> None:
        # a lot simpler when there isn't a convoluted intake sequence
        match self.state:
            case self.IntakeState.Intaking:
                self.setPivotAngle(constants.kIntakingAngle)
                self.intakeMotor.set(
                    Talon.ControlMode.Percent, -1 * constants.kIntakeMotorSpeed
                )
            case self.IntakeState.Idle:
                self.setPivotAngle(constants.kScoreAngle)
                self.intakeMotor.set(Talon.ControlMode.Percent, 0)
            case self.IntakeState.Scoring:
                self.setPivotAngle(constants.kScoreAngle)
                self.intakeMotor.set(
                    Talon.ControlMode.Percent, constants.kIntakeMotorSpeed
                )
            case self.IntakeState.Knock:
                self.setPivotAngle(constants.kKnockAngle)
                self.intakeMotor.set(Talon.ControlMode.Percent, 0)

        self.intakeAtPositionPublisher.set(self.intakeAtPosition())
        self.intakeStatePublisher.set(str(self.state))
        if RobotBase.isSimulation():
            self.pivotAnglePublisher.set(self.getPivotAngle())
        else:
            self.pivotAnglePublisher.set(
                intakeAccountForSillyEncoder(self.pivotEncoder.getPosition().radians())
            )

    def resetPivot(self) -> None:
        encoderInRadians = intakeAccountForSillyEncoder(
            self.pivotEncoder.getPosition().radians()
        )
        pivotMotorPosition = (
            encoderInRadians
            / constants.kRadiansPerRevolution
            * constants.kPivotGearRatio
        )
        self.pivotMotor.setEncoderPosition(pivotMotorPosition)

    def setPivotAngle(self, rotation: Rotation2d) -> None:
        self.targetAngle = rotation
        self.pivotMotor.set(
            Talon.ControlMode.MotionMagic,
            clamp(rotation.radians(), 0, constants.kIntakingAngle.radians())
            / constants.kRadiansPerRevolution
            * constants.kPivotGearRatio,
        )

    def getPivotAngle(self) -> float:
        return (
            self.pivotMotor.get(Talon.ControlMode.Position)
            / constants.kPivotGearRatio
            * constants.kRadiansPerRevolution
        )

    def intakeAtPosition(self) -> bool:
        return (
            abs(self.targetAngle.radians() - self.getPivotAngle())
            < constants.kIntakePivotTolerance
        )

    def setIntaking(self) -> None:
        self.state = self.IntakeState.Intaking

    def setIdle(self) -> None:
        self.state = self.IntakeState.Idle

    def setScoring(self) -> None:
        self.state = self.IntakeState.Scoring

    def setKnock(self) -> None:
        self.state = self.IntakeState.Knock
