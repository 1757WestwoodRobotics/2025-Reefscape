from commands2.command import Command
from wpilib import Timer
from subsystems.intakesubsystem import IntakeSubsystem


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


class IntakePrepScore(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setScore()


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
