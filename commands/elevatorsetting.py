from commands2 import Command
from wpilib import Timer
from subsystems.elevatorsubsystem import ElevatorSubsystem


class SetElevatorState(Command):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.elevator = elevatorSubsystem
        self.t = Timer()
        self.addRequirements(self.elevator)

    def initialize(self):
        self.t.reset()
        self.t.start()

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def isFinished(self) -> bool:
        return self.elevator.atPosition() and self.t.get() > 0.5


class ElevatorL4Position(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setL4Position()


class ElevatorL3Position(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setL3Position()


class ElevatorL2Position(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setL2Position()


class ElevatorL1Position(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setL1Position()


class ElevatorAlgaeHigh(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setAlgaeHigh()


class ElevatorAlgaeLow(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setAlgaeLow()


class ElevatorIntakePosition(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setIntakePosition()
