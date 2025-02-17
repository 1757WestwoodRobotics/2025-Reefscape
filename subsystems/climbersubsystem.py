from enum import Enum, auto
from wpilib import SmartDashboard
from commands2 import Subsystem
from math import pi

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

    def periodic(self) -> None:
        match self.state:
            case self.ClimberState.TuckedPosition:
                self.setClimberMotorAtPosition(constants.kTuckedPosition)
            case self.ClimberState.AtFramePosition:
                self.setClimberMotorAtPosition(constants.kClimberAtFramePosition)
            case self.ClimberState.EndClimbPosition:
                self.setClimberMotorAtPosition(constants.kClimberEndClimbPosition)

    def setClimberMotorAtPosition(self, winchPosition) -> None:
        self.ClimberMotor.set(
            Talon.ControlMode.Position,
            (winchPosition)
            / (constants.kWinchDiameter * pi)
            * constants.kMotorWinchGearRatio,
        )

    def setTuckedPosition(self) -> None:
        self.state = self.ClimberState.TuckedPosition

    def setAtFramePosition(self) -> None:
        self.state = self.ClimberState.AtFramePosition

    def setEndClimbPosition(self) -> None:
        self.state = self.ClimberState.EndClimbPosition
