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


class FudgeIntakeForward(FudgeIntake):
    def __init__(self, intake: IntakeSubsystem) -> None:
        FudgeIntake.__init__(self, intake)

    def execute(self) -> None:
        self.intake.intakeFudgePublisher.set(
            self.intake.intakeFudgeGetter.get() - constants.kIntakeFudgeAmount
        )


class FudgeIntakeBackward(FudgeIntake):
    def __init__(self, intake: IntakeSubsystem) -> None:
        FudgeIntake.__init__(self, intake)

    def execute(self) -> None:
        self.intake.intakeFudgePublisher.set(
            self.intake.intakeFudgeGetter.get() + constants.kIntakeFudgeAmount
        )
