from enum import Enum, auto
from commands2 import Subsystem

from ntcore import NetworkTableInstance

from util.simtalon import Talon
import constants


class ClimberSubsystem(Subsystem):
    class ClimberState(Enum):
        TuckedPosition = auto()
        AtFramePosition = auto()
        NothingPressed = auto()
        PassiveDeploy = auto()
        ManualMode = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.climberMotor = Talon(
            constants.kClimberCANID,
            constants.kClimberName,
            constants.kClimberPGain,
            constants.kClimberIGain,
            constants.kClimberDGain,
        )
        self.climberMotor.setCurrentLimit(constants.kIntakeCurrentLimit)
        self.climberMotor.setNeutralMode(Talon.NeutralMode.Brake)

        self.state = self.ClimberState.TuckedPosition
        self.targetPosition = constants.kClimberTuckedPosition

        self.climberPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberPositionKey)
            .publish()
        )
        self.climberPositionGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberPositionKey)
            .subscribe(0)
        )
        self.climberPositionTargetPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberTargetKey)
            .publish()
        )

        self.climberAtPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kClimberAtPositionKey)
            .publish()
        )

        self.climberStatePublisher = (
            NetworkTableInstance.getDefault()
            .getStringTopic(constants.kClimberStateKey)
            .publish()
        )

        self.climberEndPositionGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberEndClimbKey)
            .subscribe(constants.kClimberTuckedPosition)
        )

        self.climberEndPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberEndClimbKey)
            .publish()
        )

        self.climberEndPositionPublisher.set(constants.kClimberTuckedPosition)

        self.climberManualTargetPositionGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberManualTargetPositionKey)
            .subscribe(constants.kClimberTuckedPosition)
        )

        self.climberManualTargetPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberManualTargetPositionKey)
            .publish()
        )

        self.heldPosition = self.climberMotor.get(Talon.ControlMode.Position)

    def periodic(self) -> None:
        EndClimbPosition = self.climberEndPositionGetter.get()
        match self.state:
            case self.ClimberState.ManualMode:
                self.setClimberMotorTowardsPosition(
                    self.climberManualTargetPositionGetter.get()
                )
            case self.ClimberState.TuckedPosition:
                self.setClimberMotorTowardsPosition(EndClimbPosition)
            case self.ClimberState.AtFramePosition:
                self.setClimberMotorTowardsPosition(constants.kClimberAtFramePosition)
            case self.ClimberState.PassiveDeploy:
                self.setClimberMotorTowardsPosition(
                    constants.kClimberMiniDeployPosition
                )
            case self.ClimberState.NothingPressed:
                self.setClimberMotorTowardsPosition(self.heldPosition)

        if self.state is not self.ClimberState.ManualMode:
            self.climberManualTargetPositionPublisher.set(self.getClimberPosition())

        self.climberStatePublisher.set(str(self.state))
        self.climberPositionPublisher.set(self.getClimberPosition())
        self.climberAtPositionPublisher.set(self.climberAtPosition())
        self.climberPositionTargetPublisher.set(self.targetPosition)

    def getClimberPosition(self) -> float:
        return self.climberMotor.get(Talon.ControlMode.Position)

    def setClimberMotorTowardsPosition(self, position) -> None:
        self.targetPosition = position
        self.climberMotor.set(Talon.ControlMode.Position, self.targetPosition)

    def climberAtPosition(self) -> bool:
        return (
            abs(self.climberMotor.get(Talon.ControlMode.Position) - self.targetPosition)
            < constants.kClimberPositionTolerance
        )

    def setTuckedPosition(self) -> None:
        self.state = self.ClimberState.TuckedPosition

    def setPassivePosition(self) -> None:
        self.state = self.ClimberState.PassiveDeploy

    def setAtFramePosition(self) -> None:
        self.state = self.ClimberState.AtFramePosition

    def setManualMode(self) -> None:
        self.state = self.ClimberState.ManualMode

    def setNothingPressedPosition(self) -> None:
        if (
            self.state is not self.ClimberState.NothingPressed
        ):  # on initiating the state
            self.heldPosition = self.climberMotor.get(Talon.ControlMode.Position)
        self.state = self.ClimberState.NothingPressed
