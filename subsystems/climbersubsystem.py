from enum import Enum, auto
from commands2 import Subsystem

from ntcore import NetworkTableInstance

from util.simtalon import Talon
import constants


class ClimberSubsystem(Subsystem):
    class ClimberState(Enum):
        TuckedPosition = auto()
        AtFramePosition = auto()
        EndClimbPosition = auto()
        NothingPressed = auto()

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
        self.climberMotor.setNeutralMode(Talon.NeutralMode.Brake)

        self.state = self.ClimberState.TuckedPosition
        self.targetPosition = constants.kClimberTuckedPosition

        self.climberPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberPositionKey)
            .publish()
        )

        self.climberPositionTargetPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kClimberTargetKey)
            .publish()
        )

        self.climberStatePublisher = (
            NetworkTableInstance.getDefault()
            .getStringTopic(constants.kClimberStateKey)
            .publish()
        )
        self.heldPosition = self.climberMotor.get(Talon.ControlMode.Position)

    def periodic(self) -> None:
        match self.state:
            case self.ClimberState.TuckedPosition:
                self.setClimberMotorTowardsPosition(constants.kClimberTuckedPosition)
            case self.ClimberState.AtFramePosition:
                self.setClimberMotorTowardsPosition(constants.kClimberAtFramePosition)
            case self.ClimberState.EndClimbPosition:
                self.setClimberMotorTowardsPosition(constants.kClimberEndClimbPosition)
            case self.ClimberState.NothingPressed:
                self.setClimberMotorTowardsPosition(self.heldPosition)

        self.climberStatePublisher.set(str(self.state))
        self.climberPositionPublisher.set(
            self.climberMotor.get(Talon.ControlMode.Position)
        )
        self.climberPositionTargetPublisher.set(self.targetPosition)

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

    def setAtFramePosition(self) -> None:
        self.state = self.ClimberState.AtFramePosition

    def setEndClimbPosition(self) -> None:
        self.state = self.ClimberState.EndClimbPosition

    def setNothingPressedPosition(self) -> None:
        if (
            self.state is not self.ClimberState.NothingPressed
        ):  # on initiating the state
            self.heldPosition = self.climberMotor.get(Talon.ControlMode.Position)
        self.state = self.ClimberState.NothingPressed
