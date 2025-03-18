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

        self.heldPosition = self.climberMotor.get(Talon.ControlMode.Position)

    def periodic(self) -> None:
        EndClimbPosition = self.climberEndPositionGetter.get()
        match self.state:
            case self.ClimberState.ManualMode:
                self.setClimberMotorTowardsPosition(self.climberPositionGetter.get())
            case self.ClimberState.TuckedPosition:
                self.setClimberMotorTowardsPosition(EndClimbPosition)
            case self.ClimberState.AtFramePosition:
                self.setClimberMotorTowardsPosition(constants.kClimberAtFramePosition)
            case self.ClimberState.NothingPressed:
                self.setClimberMotorTowardsPosition(self.heldPosition)

        self.climberStatePublisher.set(str(self.state))
        if self.state is not self.ClimberState.ManualMode:
            self.climberPositionPublisher.set(
                self.climberMotor.get(Talon.ControlMode.Position)
            )
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
        self.climberManualModePublisher.set(False)

    def setAtFramePosition(self) -> None:
        self.state = self.ClimberState.AtFramePosition
        self.climberManualModePublisher.set(False)

    def setManualMode(self) -> None:
        self.state = self.ClimberState.ManualMode

    def setNothingPressedPosition(self) -> None:
        if (
            self.state is not self.ClimberState.NothingPressed
        ):  # on initiating the state
            self.heldPosition = self.climberMotor.get(Talon.ControlMode.Position)
        self.state = self.ClimberState.NothingPressed
