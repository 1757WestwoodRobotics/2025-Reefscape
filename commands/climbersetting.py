from commands2 import Command, ParallelCommandGroup
from wpilib import Timer
from subsystems.climbersubsystem import ClimberSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from commands.elevatorsetting import ElevatorDefaultL1
from util.convenientmath import clamp
import constants


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


class PrepClimb(ParallelCommandGroup):
    def __init__(self, climber: ClimberSubsystem, elevator: ElevatorSubsystem):
        ParallelCommandGroup.__init__(
            self, ClimberAtFrame(climber), ElevatorDefaultL1(elevator)
        )
        self.setName(__class__.__name__)


class ClimberNothingPressed(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem) -> None:
        SetClimberState.__init__(self, climberSubsystem)

    def execute(self) -> None:
        self.climber.setNothingPressedPosition()


class ClimberPassiveDeploy(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem) -> None:
        SetClimberState.__init__(self, climberSubsystem)

    def execute(self) -> None:
        self.climber.setPassivePosition()


class ClimberManualMode(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem) -> None:
        SetClimberState.__init__(self, climberSubsystem)

    def execute(self) -> None:
        self.climber.setManualMode()


class ClimberManualUp(Command):
    def __init__(self, climber: ClimberSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(self.climber)

    def execute(self):
        self.climber.setManualMode()
        climberPosition = self.climber.getClimberPosition()
        self.climber.climberManualTargetPositionPublisher.set(
            clamp(climberPosition + constants.kClimberManualIncrement, -10, 250)
        )


class ClimberManualDown(Command):
    def __init__(self, climber: ClimberSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(self.climber)

    def execute(self):
        self.climber.setManualMode()
        climberPosition = self.climber.getClimberPosition()
        self.climber.climberManualTargetPositionPublisher.set(
            clamp(climberPosition - constants.kClimberManualIncrement, -10, 250)
        )
