from commands2 import ParallelCommandGroup, SequentialCommandGroup
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from commands.elevatorsetting import (
    ElevatorAlgaeHigh,
    ElevatorAlgaeLow,
    ElevatorL1Position,
)
from commands.intakesetting import IntakeKnock, IntakeIdle


class AlgaeKnockHigh(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorAlgaeHigh(elevatorSubsystem),
            IntakeKnock(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeKnockLow(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorAlgaeLow(elevatorSubsystem),
            IntakeKnock(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class KnockExitSequence(SequentialCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        SequentialCommandGroup.__init__(
            self, IntakeIdle(intakeSubsystem), ElevatorL2Position(elevatorSubsystem)
        )
        self.setName(__class__.__name__)
