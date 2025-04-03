from commands2 import Command, ParallelCommandGroup
from wpilib import Timer
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from commands.elevatorsetting import ElevatorIntakePosition


class SetIntakeState(Command):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intakeSubsystem
        self.addRequirements(self.intake)

        self.t = Timer()

    def initialize(self):
        self.t.reset()
        self.t.start()

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def isFinished(self) -> bool:
        return self.intake.intakeAtPosition() and self.t.get() > 0.1


class IntakeIdle(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setIdle()


class IntakeScoring(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setScoring()


class IntakeKnock(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setKnock()


class IntakeCoral(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setIntaking()

    def isFinished(self) -> bool:
        return self.intake.hasGamepiece() and self.t.get() > 0.1


class IntakeCoralProcess(ParallelCommandGroup):
    def __init__(
        self, intakeSubsystem: IntakeSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        ParallelCommandGroup.__init__(
            self,
            ElevatorIntakePosition(elevatorSubsystem),
            IntakeCoral(intakeSubsystem),
        )
        self.setName(__class__.__name__)
