from enum import Enum, auto
from commands2 import Subsystem
from wpimath.geometry import Rotation2d
from util.simtalon import Talon
from util.simcoder import CTREEncoder
from util.angleoptimize import intakeAccountForSillyEncoder
from ntcore import NetworkTableInstance

import constants


class IntakeSubsystem(Subsystem):
    class IntakeState(Enum):
        Intaking = auto()
        Score = auto()
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
        self.intakeMotor.setCurrentLimit(80)
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

        self.state = self.IntakeState.Score

        self.intakeAtPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kIntakeAtPositionKey)
            .publish()
        )

    def periodic(self) -> None:
        # a lot simpler when there isn't a convoluted intake sequence
        if self.state == self.IntakeState.Intaking:
            self.setPivotAngle(constants.kIntakingAngle)
            self.intakeMotor.set(
                Talon.ControlMode.Percent, -1 * constants.kIntakeMotorSpeed
            )
        elif self.state == self.IntakeState.Score:
            self.setPivotAngle(constants.kScoreAngle)
            self.intakeMotor.set(Talon.ControlMode.Percent, 0)
        elif self.state == self.IntakeState.Scoring:
            self.setPivotAngle(constants.kScoreAngle)
            self.intakeMotor.set(Talon.ControlMode.Percent, constants.kIntakeMotorSpeed)
        elif self.state == self.IntakeState.Knock:
            self.setPivotAngle(constants.kKnockAngle)
            self.intakeMotor.set(Talon.ControlMode.Percent, 0)

        self.intakeAtPositionPublisher.set(self.intakeAtPosition())

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
            rotation.radians()
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