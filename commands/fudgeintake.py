from commands2 import Command

from subsystems.intakesubsystem import IntakeSubsystem
import constants


class FudgeIntake(Command):
    def __init__(self, intake: IntakeSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intake

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self):
        raise NotImplementedError("Must be implemented by subclass")

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True


class FudgeIntakeScoreForward(FudgeIntake):
    def __init__(self, intake: IntakeSubsystem) -> None:
        FudgeIntake.__init__(self, intake)

    def execute(self) -> None:
        self.intake.intakeFudgeScorePublisher.set(
            self.intake.intakeFudgeScoreGetter.get() - constants.kIntakeFudgeAmount
        )


class FudgeIntakeScoreBackward(FudgeIntake):
    def __init__(self, intake: IntakeSubsystem) -> None:
        FudgeIntake.__init__(self, intake)

    def execute(self) -> None:
        self.intake.intakeFudgeScorePublisher.set(
            self.intake.intakeFudgeScoreGetter.get() + constants.kIntakeFudgeAmount
        )


class FudgeIntakeCoralUp(FudgeIntake):
    def __init__(self, intake: IntakeSubsystem) -> None:
        FudgeIntake.__init__(self, intake)

    def execute(self) -> None:
        self.intake.intakeFudgeCoralPublisher.set(
            self.intake.intakeFudgeCoralGetter.get() - constants.kIntakeFudgeAmount
        )


class FudgeIntakeCoralDown(FudgeIntake):
    def __init__(self, intake: IntakeSubsystem) -> None:
        FudgeIntake.__init__(self, intake)

    def execute(self) -> None:
        self.intake.intakeFudgeCoralPublisher.set(
            self.intake.intakeFudgeCoralGetter.get() + constants.kIntakeFudgeAmount
        )
