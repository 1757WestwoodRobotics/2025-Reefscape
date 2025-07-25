from commands2 import ParallelCommandGroup
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from commands.elevatorsetting import (
    ElevatorL1Position,
    ElevatorL4Position,
)
from commands.intakesetting import IntakeAlgae, IntakeIdle

class AlgaeScoreNet(ParallelCommandGroup):
    def __init__ (
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self, 
            ElevatorL4Position(elevatorSubsystem),
            # AlgaeScoreNet(intakeSubsystem)
        )
        self.setName(__class__.__name__)

class AlgaeScoreProcessor(ParallelCommandGroup):
    def __init__ (
            self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorL1Position(elevatorSubsystem),
            # AlgaeScoreNet(intakeSubsystem)
        )
        self.setName(__class__.__name__)