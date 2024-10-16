from commands2 import Command
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController
from subsystems.climbersubsystem import ClimberSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem


import constants


class SetClimberState(Command):
    def __init__(self, climberSubsystem: ClimberSubsystem, elevator: ElevatorSubsystem):
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climberSubsystem
        self.elevator = elevator
        self.addRequirements(self.climber, self.elevator)

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def isFinished(self) -> bool:
        return False


class ExtendClimberPosition(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem, elevator: ElevatorSubsystem):
        SetClimberState.__init__(self, climberSubsystem, elevator)

        self.targetPosition = 0
        self.controller = ProfiledPIDController(
            constants.kProfiledControllerPGain,
            constants.kProfiledControllerIGain,
            constants.kProfiledControllerDGain,
            TrapezoidProfile.Constraints(
                constants.kProfiledMaxVelocityExtend, constants.kProfiledMaxAccleration
            ),
        )

    def initialize(self):
        self.targetPosition = constants.kClimbingRetractedHeight
        self.controller.reset(self.targetPosition)

    def execute(self) -> None:
        self.targetPosition += self.controller.calculate(
            self.targetPosition, constants.kClimbingTopHeight
        )
        self.climber.setClimberTargetPosition(
            self.targetPosition + constants.kClimberHeightOffset
        )
        self.elevator.setTargetPosition(
            self.targetPosition + constants.kClimberHeightExtendOffsetElevator
        )


class RetractClimberPosition(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem, elevator: ElevatorSubsystem):
        SetClimberState.__init__(self, climberSubsystem, elevator)

        self.targetPosition = 0
        self.controller = ProfiledPIDController(
            constants.kProfiledControllerPGain,
            constants.kProfiledControllerIGain,
            constants.kProfiledControllerDGain,
            TrapezoidProfile.Constraints(
                constants.kProfiledMaxVelocityRetract, constants.kProfiledMaxAccleration
            ),
        )

    def initialize(self):
        self.targetPosition = constants.kClimbingTopHeight
        self.controller.reset(self.targetPosition)

    def execute(self) -> None:
        self.targetPosition += self.controller.calculate(
            self.targetPosition, constants.kClimbingRetractedHeight
        )
        self.climber.setClimberTargetPosition(
            self.targetPosition + constants.kClimberHeightOffset
        )
        self.elevator.setTargetPosition(
            self.targetPosition - constants.kClimberFloorTargetElevatorExtra
        )


class NeutralClimberState(SetClimberState):
    def __init__(
        self, climberSubsystem: ClimberSubsystem, elevatorSubsystem: ElevatorSubsystem
    ):
        SetClimberState.__init__(self, climberSubsystem, elevatorSubsystem)

    def execute(self) -> None:
        self.climber.setClimberHold()

    def isFinished(self) -> bool:
        return True
