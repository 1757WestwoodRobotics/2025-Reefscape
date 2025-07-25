from commands2 import ParallelCommandGroup, SequentialCommandGroup
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from commands.elevatorsetting import (
    ElevatorAlgaeRemovalHigh,
    ElevatorAlgaeRemovalLow,
    ElevatorL2Position,
    ElevatorL1Position,
)
from commands.intakesetting import IntakeAlgae, IntakeIdle


class AlgaeRemovalHigh(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorAlgaeRemovalHigh(elevatorSubsystem),
            IntakeAlgae(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeRemovalLow(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorAlgaeRemovalLow(elevatorSubsystem),
            IntakeAlgae(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeGroundIntake(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self, 
            ElevatorL1Position(elevatorSubsystem),
            # GroundAlgaeIntake(intakeSubsystem),
        )
        self.setName(__class__.__name__)

class AlgaeLollipopIntake(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self, 
            # ElevatorLollipopPosition(elevatorSubsystem),
            # GroundAlgaeIntake(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeRemovalExitSequence(SequentialCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        SequentialCommandGroup.__init__(
            self, IntakeIdle(intakeSubsystem), ElevatorL2Position(elevatorSubsystem)
        )
        self.setName(__class__.__name__)