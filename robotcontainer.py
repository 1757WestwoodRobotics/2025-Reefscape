import os
import wpilib
from wpimath.geometry import Pose2d
import commands2
from commands2.button import POVButton
from pathplannerlib.auto import PathPlannerAuto, NamedCommands
from commands.drive.absoluterelativedrive import AbsoluteRelativeDrive
from commands.drive.anglealign import AngleAlignDrive
from commands.drive.drivewaypoint import DriveToReefPosition, SetLeftReef, SetRightReef
from commands.resetdrive import ResetDrive
from commands.drivedistance import DriveDistance
from commands.defensestate import DefenseState
from commands.intakesetting import (
    IntakeIdle,
    IntakeScoring,
    IntakeCoralProcess,
)
from commands.elevatorsetting import (
    ElevatorIntakePositionToggleOn,
    ElevatorIntakePositionToggleOff,
    ElevatorL1Position,
    ElevatorL2Position,
    ElevatorL3Position,
    ElevatorL4Position,
    ElevatorAlgaeHigh,
    ElevatorAlgaeLow,
    ElevatorManualUp,
    ElevatorManualDown,
    ElevatorManualMode,
    SetNoSpace,
    SetCoralSpace,
)
from commands.climbersetting import (
    ClimberAtFrame,
    ClimberTucked,
    ClimberNothingPressed,
    ClimberManualUp,
    ClimberManualDown,
)
from commands.fudgeelevator import FudgeElevatorUp, FudgeElevatorDown
from commands.fudgeintake import (
    FudgeIntakeScoreForward,
    FudgeIntakeScoreBackward,
    FudgeIntakeCoralUp,
    FudgeIntakeCoralDown,
)
from commands.algaeknock import AlgaeKnockHigh, AlgaeKnockLow, KnockExitSequence

# from commands.drive.drivewaypoint import DriveWaypoint
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.loggingsubsystem import LoggingSubsystem
from subsystems.vision.visionsubsystem import VisionSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from subsystems.climbersubsystem import ClimberSubsystem

from operatorinterface import OperatorInterface
from util.helpfultriggerwrappers import ModifiableJoystickButton, NetworkTableButton

import constants


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The operator interface (driver controls)
        self.operatorInterface = OperatorInterface()

        # The robot's subsystems
        self.drive = DriveSubsystem()
        self.vision = VisionSubsystem(self.drive)
        self.log = LoggingSubsystem(self.operatorInterface)
        self.intake = IntakeSubsystem()
        self.elevator = ElevatorSubsystem()
        self.climber = ClimberSubsystem()

        # Robot demo subsystems
        # self.velocity = VelocityControl()

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = commands2.SequentialCommandGroup(
            ResetDrive(self.drive),
            DriveDistance(
                -4 * constants.kWheelCircumference,
                0.2,
                DriveDistance.Axis.X,
                self.drive,
            ),
        )
        self.nothingAuto = commands2.WaitCommand(constants.kAutoDuration)

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        NamedCommands.registerCommand("elevatorL1", ElevatorL1Position(self.elevator))
        NamedCommands.registerCommand("elevatorL2", ElevatorL2Position(self.elevator))
        NamedCommands.registerCommand("elevatorL3", ElevatorL3Position(self.elevator))
        NamedCommands.registerCommand("elevatorL4", ElevatorL4Position(self.elevator))
        NamedCommands.registerCommand(
            "elevatorAlgaeLow", ElevatorAlgaeLow(self.elevator)
        )
        NamedCommands.registerCommand(
            "elevatorAlgaeHigh", ElevatorAlgaeHigh(self.elevator)
        )
        NamedCommands.registerCommand(
            "intakeCoral", IntakeCoralProcess(self.elevator, self.intake)
        )
        NamedCommands.registerCommand(
            "visionAlign", DriveToReefPosition(self.drive, lambda: 0, lambda: 0)
        )
        NamedCommands.registerCommand("intakeIdle", IntakeIdle(self.intake))
        NamedCommands.registerCommand("intakeScoring", IntakeScoring(self.intake))
        # NamedCommands.registerCommand(
        #     "elevatorIntake", ElevatorIntakePosition(self.elevator)
        # )
        NamedCommands.registerCommand(
            "intakeCoral", IntakeCoralProcess(self.intake, self.elevator)
        )
        NamedCommands.registerCommand("intakeIdle", IntakeIdle(self.intake))
        NamedCommands.registerCommand("intakeScoring", IntakeScoring(self.intake))
        NamedCommands.registerCommand("leftReef", SetLeftReef(self.drive))
        NamedCommands.registerCommand("rightReef", SetRightReef(self.drive))

        pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        for file in os.listdir(pathsPath):
            relevantName = file.split(".")[0]
            auton = PathPlannerAuto(relevantName)
            wpilib.SmartDashboard.putData(f"autos/{relevantName}", auton)
            self.chooser.addOption(relevantName, auton)

        self.chooser.addOption("Do Nothing Auto", self.nothingAuto)
        self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            AbsoluteRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kTurboSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kTurboSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
                self.operatorInterface.chassisControls.rotationY,
            )
        )
        self.elevator.setDefaultCommand(ElevatorL2Position(self.elevator))

        self.intake.setDefaultCommand(IntakeIdle(self.intake))

        self.climber.setDefaultCommand(ClimberNothingPressed(self.climber))

        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # ModifiableJoystickButton(
        #     self.operatorInterface.fieldRelativeCoordinateModeControl
        # ).toggleOnTrue(
        #     RobotRelativeDrive(
        #         self.drive,
        #         self.operatorInterface.chassisControls.forwardsBackwards,
        #         self.operatorInterface.chassisControls.sideToSide,
        #         self.operatorInterface.chassisControls.rotationX,
        #     )
        # )

        ModifiableJoystickButton(self.operatorInterface.alignAngle).whileTrue(
            AngleAlignDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kNormalSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kNormalSpeedMultiplier,
            ).repeatedly()
        )
        ModifiableJoystickButton(self.operatorInterface.autoWaypoint).whileTrue(
            DriveToReefPosition(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
            ).repeatedly()
        )

        ModifiableJoystickButton(self.operatorInterface.resetGyro).onTrue(
            ResetDrive(self.drive, Pose2d(0, 0, 0))
        )

        ModifiableJoystickButton(self.operatorInterface.defenseStateControl).whileTrue(
            DefenseState(self.drive)
        )

        ModifiableJoystickButton(self.operatorInterface.intakeCoral).whileTrue(
            IntakeCoralProcess(self.intake, self.elevator).repeatedly()
        )
        ModifiableJoystickButton(self.operatorInterface.intakeScoring).whileTrue(
            IntakeScoring(self.intake).repeatedly()
        )
        ModifiableJoystickButton(self.operatorInterface.elevatorL1).whileTrue(
            ElevatorL1Position(self.elevator).repeatedly()
        )
        ModifiableJoystickButton(self.operatorInterface.elevatorL2).whileTrue(
            ElevatorL2Position(self.elevator).repeatedly()
        )
        ModifiableJoystickButton(self.operatorInterface.elevatorL3).whileTrue(
            ElevatorL3Position(self.elevator).repeatedly()
        )
        ModifiableJoystickButton(self.operatorInterface.elevatorL4).whileTrue(
            ElevatorL4Position(self.elevator).repeatedly()
        )

        POVButton(*self.operatorInterface.algaeLow).whileTrue(
            AlgaeKnockLow(self.intake, self.elevator).repeatedly()
        ).onFalse(KnockExitSequence(self.intake, self.elevator).repeatedly())
        POVButton(*self.operatorInterface.algaeLow2).whileTrue(
            AlgaeKnockLow(self.intake, self.elevator).repeatedly()
        ).onFalse(KnockExitSequence(self.intake, self.elevator).repeatedly())
        POVButton(*self.operatorInterface.algaeLow3).whileTrue(
            AlgaeKnockLow(self.intake, self.elevator).repeatedly()
        ).onFalse(KnockExitSequence(self.intake, self.elevator).repeatedly())

        POVButton(*self.operatorInterface.algaeHigh).whileTrue(
            AlgaeKnockHigh(self.intake, self.elevator).repeatedly()
        ).onFalse(KnockExitSequence(self.intake, self.elevator).repeatedly())
        POVButton(*self.operatorInterface.algaeHigh2).whileTrue(
            AlgaeKnockHigh(self.intake, self.elevator).repeatedly()
        ).onFalse(KnockExitSequence(self.intake, self.elevator).repeatedly())
        POVButton(*self.operatorInterface.algaeHigh3).whileTrue(
            AlgaeKnockHigh(self.intake, self.elevator).repeatedly()
        ).onFalse(KnockExitSequence(self.intake, self.elevator).repeatedly())

        ModifiableJoystickButton(
            self.operatorInterface.elevatorIntakePositionToggleOn
        ).onTrue(ElevatorIntakePositionToggleOn(self.elevator))

        ModifiableJoystickButton(
            self.operatorInterface.elevatorIntakePositionToggleOff
        ).onTrue(ElevatorIntakePositionToggleOff(self.elevator))

        ModifiableJoystickButton(self.operatorInterface.climberUp).onTrue(
            ClimberAtFrame(self.climber)
        )

        ModifiableJoystickButton(self.operatorInterface.climberDown).onTrue(
            ClimberTucked(self.climber)
        )

        # manual elevator
        NetworkTableButton(constants.kElevatorManualModeKey).whileTrue(
            ElevatorManualMode(self.elevator).repeatedly()
        )

        ModifiableJoystickButton(self.operatorInterface.elevatorManualUp).whileTrue(
            ElevatorManualUp(self.elevator).repeatedly()
        )

        ModifiableJoystickButton(self.operatorInterface.elevatorManualDown).whileTrue(
            ElevatorManualDown(self.elevator).repeatedly()
        )

        # manual climber
        ModifiableJoystickButton(self.operatorInterface.climberManualUp).whileTrue(
            ClimberManualUp(self.climber).repeatedly()
        )

        ModifiableJoystickButton(self.operatorInterface.climberManualDown).whileTrue(
            ClimberManualDown(self.climber).repeatedly()
        )

        ModifiableJoystickButton(self.operatorInterface.elevatorFudgeUp).onTrue(
            FudgeElevatorUp(self.elevator)
        )

        ModifiableJoystickButton(self.operatorInterface.elevatorFudgeDown).onTrue(
            FudgeElevatorDown(self.elevator)
        )

        ModifiableJoystickButton(self.operatorInterface.intakeFudgeScoreForward).onTrue(
            FudgeIntakeScoreForward(self.intake)
        )

        ModifiableJoystickButton(
            self.operatorInterface.intakeFudgeScoreBackward
        ).onTrue(FudgeIntakeScoreBackward(self.intake))

        ModifiableJoystickButton(self.operatorInterface.intakeFudgeCoralUp).onTrue(
            FudgeIntakeCoralUp(self.intake)
        )

        ModifiableJoystickButton(self.operatorInterface.intakeFudgeCoralDown).onTrue(
            FudgeIntakeCoralDown(self.intake)
        )

        ModifiableJoystickButton(self.operatorInterface.setLeftReef).onTrue(
            SetLeftReef(self.drive)
        )

        ModifiableJoystickButton(self.operatorInterface.setRightReef).onTrue(
            SetRightReef(self.drive)
        )

        ModifiableJoystickButton(self.operatorInterface.setNoSpace).onTrue(
            SetNoSpace(self.elevator)
        )

        ModifiableJoystickButton(self.operatorInterface.setCoralSpace).onTrue(
            SetCoralSpace(self.elevator)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
