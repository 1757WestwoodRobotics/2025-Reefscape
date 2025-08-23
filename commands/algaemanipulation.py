from commands2 import ParallelCommandGroup, SequentialCommandGroup
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from commands.elevatorsetting import (
    ElevatorAlgaeRemovalHigh,
    ElevatorAlgaeRemovalLow,
    ElevatorL1Position,
    ElevatorL4Position,
    ElevatorAlgaeIdlePosition,
    ElevatorLollipopAlgae,
)
from commands.intakesetting import IntakeAlgae, IntakeIdle, GroundAlgaeIntake


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
            GroundAlgaeIntake(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeLollipopIntake(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorLollipopAlgae(elevatorSubsystem),
            GroundAlgaeIntake(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeScoreNet(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorL4Position(elevatorSubsystem),
            AlgaeScoreNet(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeScoreProcessor(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorL1Position(elevatorSubsystem),
            AlgaeScoreProcessor(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeIntakeExitSequence(SequentialCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        SequentialCommandGroup.__init__(
            self,
            ElevatorAlgaeIdlePosition(elevatorSubsystem),
            IntakeIdle(intakeSubsystem),
        )
        self.setName(__class__.__name__)


class AlgaeGroundExitSequence(SequentialCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        SequentialCommandGroup.__init__(
            self,
            ElevatorAlgaeIdlePosition(elevatorSubsystem),
            IntakeIdle(intakeSubsystem),
        )
