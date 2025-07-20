import constants
from math import pi
from commands2 import Subsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from util.simtalon import Talon


def __init__(self) -> None:
    self.algaeIntakeMotor1 = Talon(
        constants.kIntakeCANID,
        constants.kIntakeName,
        constants.kIntakePGain,
        constants.kIntakeIGain,
        constants.kIntakeDGain,
        constants.kIntakeInverted,
    )