from enum import Enum, auto
from commands2 import Subsystem
from math import inf

from ntcore import NetworkTableInstance

from util.simtalon import Talon
import constants


class ClimberSubsystem(Subsystem):
    class ClimberState(Enum):
        TuckedPosition = auto()
        AtFramePosition = auto()
        EndClimbPosition = auto()

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

        self.state = self.ClimberState.TuckedPosition
        self.targetPosition = inf

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
            .getFloatTopic(constants.kClimberStateKey)
            .publish()
        )

    def periodic(self) -> None:
        match self.state:
            case self.ClimberState.TuckedPosition:
                self.setClimberMotorAtPosition(constants.kClimberTuckedPosition)
            case self.ClimberState.AtFramePosition:
                self.setClimberMotorAtPosition(constants.kClimberAtFramePosition)
            case self.ClimberState.EndClimbPosition:
                self.setClimberMotorAtPosition(constants.kClimberEndClimbPosition)

        self.climberStatePublisher.set(str(self.state))
        self.climberPositionPublisher.set(
            self.climberMotor.get(Talon.ControlMode.Position)
        )
        self.climberPositionTargetPublisher.set(self.targetPosition)

    def setClimberMotorAtPosition(self, winchPosition) -> None:
        self.targetPosition = (
            winchPosition
            / constants.kClimberWinchCircumferance
            * constants.kClimberGearRatio
        )

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
