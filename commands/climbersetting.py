from commands2.command import Command
from wpilib import Timer
from subsystems.climbersubsystem import ClimberSubsystem


class SetClimberState(Command):
    def __init__(self, climberSubsystem: ClimberSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climberSubsystem
        self.addRequirements(self.climber)

        self.t = Timer()

    def initialize(self):
        self.t.reset()
        self.t.start()

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def isFinished(self) -> bool:
        return self.climber.climberAtPosition() and self.t.get() > 0.1


class ClimberTucked(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem) -> None:
        SetClimberState.__init__(self, climberSubsystem)

    def execute(self) -> None:
        self.climber.setTuckedPosition()


class ClimberAtFrame(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem) -> None:
        SetClimberState.__init__(self, climberSubsystem)

    def execute(self) -> None:
        self.climber.setAtFramePosition()
