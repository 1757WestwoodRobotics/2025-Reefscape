from commands2.command import Command
from wpilib import SmartDashboard
from wpilib import Timer
from subsystems.intakesubsystem import IntakeSubsystem

import constants


class SetIntakeState(
    Command
):  # since all ball subsystem commands are the same, subclasses will implement calling while this serves as the base
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


class FloorIntake(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setIntaking()

    # def isFinished(self) -> bool:
    #     return self.intake.intakeAtPosition() and self.t.get() > 0.1 and self.intake.putInPlace


class HoldIntakeAtHandoff(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setHolding()


class FeedIntakeToShooter(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setFeeding()

    def isFinished(self) -> bool:
        return not SmartDashboard.getBoolean(
            constants.kIntakeHasNoteKey, True
        )  # command is finished when no note is detected


class StageIntake(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)
        self.t = Timer()

    def initialize(self):
        self.t.reset()
        self.t.start()

    def execute(self) -> None:
        self.intake.setStaging()

    def isFinished(self) -> bool:
        return (
            self.intake.intakeAtPosition() and self.t.get() > 0.1
        )  # allow debounce time


class EjectInAmp(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setAmp()


class EjectInTrap(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setTrap()

    def isFinished(self) -> bool:
        return False


class ResetIntake(Command):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intakeSubsystem
        self.addRequirements(self.intake)

    def execute(self) -> None:
        self.intake.resetPivot()
