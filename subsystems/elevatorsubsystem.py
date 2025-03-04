from enum import Enum, auto
from math import pi
from commands2 import Subsystem
from ntcore import NetworkTableInstance

from util.simtalon import Talon
from util.convenientmath import clamp
import constants


class ElevatorSubsystem(Subsystem):
    class ElevatorState(Enum):
        L4Position = auto()
        L3Position = auto()
        L2Position = auto()
        L1Position = auto()
        AlgaeHigh = auto()
        AlgaeLow = auto()
        IntakePosition = auto()
        ManualMode = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)  # basic subsystem boilerplate

        self.elevatorMotor1 = Talon(
            constants.kElevator1CANID,
            constants.kElevator1Name,
            constants.kElevator1PGain,
            constants.kElevator1IGain,
            constants.kElevator1DGain,
            constants.kElevator1Inverted,
            moMagicAccel=constants.kElevatorMaxAccel,
            moMagicVel=constants.kElevatorMaxVel,
        )

        self.elevatorMotor2 = Talon(
            constants.kElevator2CANID,
            constants.kElevator2Name,
            constants.kElevator2PGain,
            constants.kElevator2IGain,
            constants.kElevator2DGain,
            constants.kElevator2Inverted,
        )
        self.elevatorMotor1.setNeutralMode(Talon.NeutralMode.Brake)
        self.elevatorMotor2.setNeutralMode(Talon.NeutralMode.Brake)
        self.elevatorMotor1.setCurrentLimit(constants.kElevatorCurrentLimit)
        self.elevatorMotor2.setCurrentLimit(constants.kElevatorCurrentLimit)

        self.elevatorMotor2.follow(self.elevatorMotor1, True)
        self.targetPosition = 0

        self.state = self.ElevatorState.L1Position

        self.elevatorStatePublisher = (
            NetworkTableInstance.getDefault()
            .getStringTopic(constants.kElevatorStateKey)
            .publish()
        )
        self.elevatorPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kElevatorPositionKey)
            .publish()
        )
        self.elevatorPositionGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kElevatorPositionKey)
            .subscribe(constants.kIntakePositionBeltPosition)
        )
        self.elevatorAtPositionPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kElevatorAtPositionKey)
            .publish()
        )
        self.elevatorFudgePublisher = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kElevatorFudgeKey)
            .publish()
        )
        self.elevatorFudgeGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kElevatorFudgeKey)
            .subscribe(0)
        )
        self.elevatorManualModePublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kElevatorManualModeKey)
            .publish()
        )
        self.elevatorManualModePublisher.set(False)

    def periodic(self) -> None:
        match self.state:
            case self.ElevatorState.ManualMode:
                self.setElevatorMotorsAtPosition(self.elevatorPositionGetter.get())
            case self.ElevatorState.L4Position:
                self.setElevatorMotorsAtPosition(constants.kL4PositionBeltPosition)
            case self.ElevatorState.L3Position:
                self.setElevatorMotorsAtPosition(constants.kL3PositionBeltPosition)
            case self.ElevatorState.L2Position:
                self.setElevatorMotorsAtPosition(constants.kL2PositionBeltPosition)
            case self.ElevatorState.L1Position:
                self.setElevatorMotorsAtPosition(constants.kL1PositionBeltPosition)
            case self.ElevatorState.AlgaeHigh:
                self.setElevatorMotorsAtPosition(constants.kAlgaeHighBeltPosition)
            case self.ElevatorState.AlgaeLow:
                self.setElevatorMotorsAtPosition(constants.kAlgaeLowBeltPosition)
            case self.ElevatorState.IntakePosition:
                self.setElevatorMotorsAtPosition(constants.kIntakePositionBeltPosition)

        self.elevatorStatePublisher.set(str(self.state))
        if self.state is not self.ElevatorState.ManualMode:
            self.elevatorPositionPublisher.set(self.getElevatorPosition())
        self.elevatorAtPositionPublisher.set(self.atPosition())

    def setElevatorMotorsAtPosition(self, beltPosition) -> None:
        self.targetPosition = beltPosition + self.elevatorFudgeGetter.get()
        self.elevatorMotor1.set(
            Talon.ControlMode.MotionMagic,
            clamp(
                beltPosition,
                0,
                constants.kL4PositionBeltPosition,
            )
            / (constants.kPulleyGearPitchDiameter * pi)
            * constants.kMotorPulleyGearRatio,
        )

    def atPosition(self) -> bool:
        return (
            abs(self.getElevatorPosition() - self.targetPosition)
            < constants.kElevatorTolerance
        )

    def getElevatorPosition(self) -> float:
        """returns in meters from bottom position"""
        return (
            self.elevatorMotor1.get(Talon.ControlMode.Position)
            / constants.kMotorPulleyGearRatio
            * constants.kPulleyGearPitchDiameter
            * pi
        )

    def setL4Position(self) -> None:
        self.state = self.ElevatorState.L4Position
        self.elevatorManualModePublisher.set(False)

    def setL3Position(self) -> None:
        self.state = self.ElevatorState.L3Position
        self.elevatorManualModePublisher.set(False)

    def setL2Position(self) -> None:
        self.state = self.ElevatorState.L2Position
        self.elevatorManualModePublisher.set(False)

    def setL1Position(self) -> None:
        self.state = self.ElevatorState.L1Position
        self.elevatorManualModePublisher.set(False)

    def setAlgaeHigh(self) -> None:
        self.state = self.ElevatorState.AlgaeHigh
        self.elevatorManualModePublisher.set(False)

    def setAlgaeLow(self) -> None:
        self.state = self.ElevatorState.AlgaeLow
        self.elevatorManualModePublisher.set(False)

    def setIntakePosition(self) -> None:
        self.state = self.ElevatorState.IntakePosition
        self.elevatorManualModePublisher.set(False)

    def setManualMode(self) -> None:
        self.state = self.ElevatorState.ManualMode
