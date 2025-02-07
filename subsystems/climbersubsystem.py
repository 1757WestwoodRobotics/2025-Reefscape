from enum import Enum, auto
from wpilib import SmartDashboard
from commands2 import Subsystem
from math import pi

from util.simtalon import Talon
import constants


class ClimberSubsystem(Subsystem):
    class ClimberState(Enum):
        IdlePosition = auto()
        ClimbPosition = auto()

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

        self.state = self.ClimberState.IdlePosition

    def periodic(self) -> None:
        if self.state == self.ClimberState.IdlePosition:
            self.setClimberMotorAtPosition(constants.kClimberIdlePosition)

        elif self.state == self.ClimberState.ClimbPosition:
            self.setClimberMotorAtPosition(constants.kClimberClimbPosition)

    def setClimberMotorAtPosition(self, winchPosition) -> None:
        self.ClimberMotor.set(
            Talon.ControlMode.Position,
            (winchPosition)
            / (constants.kWinchDiameter * pi)
            * constants.kMotorWinchGearRatio,
        )

    def setIdlePosition(self) -> None:
        self.state = self.ClimberState.IdlePosition

    def setClimbPosition(self) -> None:
        self.state = self.ClimberState.ClimbPosition
