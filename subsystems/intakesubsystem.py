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
        if RobotBase.isReal():
            self.pivotMotor.setCurrentLimit(constants.kPivotCurrentLimit)
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

        self.intakeL1SpeedPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kIntakeL1SpeedKey)
            .publish()
        )
        self.intakeL1SpeedPublisher.set(constants.kIntakeL1MotorSpeed)

        self.intakeL1SpeedGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kIntakeL1SpeedKey)
            .subscribe(constants.kIntakeL1MotorSpeed)
        )

        self.intakeL2ThroughL4Publisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kIntakeL2ThroughL4SpeedKey)
            .publish()
        )
        self.intakeL2ThroughL4Publisher.set(constants.kIntakeL2ThroughL4MotorSpeed)

        self.intakeL2ThroughL4SpeedGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kIntakeL2ThroughL4SpeedKey)
            .subscribe(constants.kIntakeL2ThroughL4MotorSpeed)
        )

        self.intakeCoralSpeedPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kIntakeCoralKey)
            .publish()
        )
        self.intakeCoralSpeedPublisher.set(constants.kIntakeMotorSpeed)

        self.intakeCoralSpeedGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kIntakeCoralKey)
            .subscribe(constants.kIntakeMotorSpeed)
        )

        self.elevatorPositionGetter = (
            NetworkTableInstance.getDefault()
            .getStringTopic(constants.kElevatorStateKey)
            .subscribe("ElevatorState.L4Position")
        )

        self.intakeFudgePublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kIntakeFudgeKey)
            .publish()
        )

        self.intakeFudgePublisher.set(0)

        self.intakeFudgeGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kIntakeFudgeKey)
            .subscribe(0)
        )

    def periodic(self) -> None:
        # a lot simpler when there isn't a convoluted intake sequence
        L1Speed = self.intakeL1SpeedGetter.get()
        L2ThroughL4Speed = self.intakeL2ThroughL4SpeedGetter.get()
        IntakeCoralSpeed = self.intakeCoralSpeedGetter.get()
        ElevatorState = self.elevatorPositionGetter.get()

        match self.state:
            case self.IntakeState.Intaking:
                self.setPivotAngle(constants.kIntakingAngle)
                self.intakeMotor.set(Talon.ControlMode.Percent, -1 * IntakeCoralSpeed)
            case self.IntakeState.Idle:
                self.setPivotAngle(constants.kScoreAngle)
                self.intakeMotor.set(Talon.ControlMode.Percent, 0)
            case self.IntakeState.Scoring:
                self.setPivotAngle(constants.kScoreAngle)
                if ElevatorState == "ElevatorState.L1Position":
                    self.intakeMotor.set(Talon.ControlMode.Percent, L1Speed)
                else:
                    self.intakeMotor.set(Talon.ControlMode.Percent, L2ThroughL4Speed)
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
        self.targetAngle = rotation + Rotation2d.fromDegrees(
            self.intakeFudgeGetter.get()
        )
        self.pivotMotor.set(
            Talon.ControlMode.MotionMagic,
            clamp(self.targetAngle.radians(), 0, constants.kIntakingAngle.radians())
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
