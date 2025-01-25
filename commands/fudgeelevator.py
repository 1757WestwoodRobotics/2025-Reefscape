from commands2 import Command

from subsystems.elevatorsubsystem import ElevatorSubsystem
import constants


class FudgeElevator(Command):
    def __init__(self, elevator: ElevatorSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.elevator = elevator

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self):
        raise NotImplementedError("Must be implemented by subclass")

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True


class FudgeElevatorDown(FudgeElevator):
    def __init__(self, elevator: ElevatorSubsystem) -> None:
        FudgeElevator.__init__(self, elevator)

    def execute(self) -> None:
        self.elevator.elevatorFudgePublisher.set(
            self.elevator.elevatorFudgeGetter.get() - constants.kElevatorFudgeAmount
        )


class FudgeElevatorUp(FudgeElevator):
    def __init__(self, elevator: ElevatorSubsystem) -> None:
        FudgeElevator.__init__(self, elevator)

    def execute(self) -> None:
        self.elevator.elevatorFudgePublisher.set(
            self.elevator.elevatorFudgeGetter.get() + constants.kElevatorFudgeAmount
        )
