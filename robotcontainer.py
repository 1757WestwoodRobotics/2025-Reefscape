import os
import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.button
from pathplannerlib.auto import (
    PathPlannerAuto,
)
from commands.drive.absoluterelativedrive import AbsoluteRelativeDrive
from commands.resetdrive import ResetDrive
from commands.drivedistance import DriveDistance
from commands.defensestate import DefenseState
from commands.intakesetting import (
    IntakeCoral,
    IntakeKnock,
    IntakeIdle,
    IntakeScoring,
)

# from commands.drive.drivewaypoint import DriveWaypoint
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.loggingsubsystem import LoggingSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.intakesubsystem import IntakeSubsystem

from operatorinterface import OperatorInterface
from util.helpfultriggerwrappers import ModifiableJoystickButton

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
        self.vision = VisionSubsystem()
        self.drive = DriveSubsystem(self.vision)
        self.log = LoggingSubsystem(self.operatorInterface)
        self.intake = IntakeSubsystem()
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

        self.intake.setDefaultCommand(IntakeIdle(self.intake))

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

        # ModifiableJoystickButton(self.operatorInterface.alignClosestWaypoint).whileTrue(
        #     DriveWaypoint(self.drive)
        # )

        ModifiableJoystickButton(self.operatorInterface.resetGyro).onTrue(
            ResetDrive(self.drive, Pose2d(1.37, 5.49, 0))
        )

        ModifiableJoystickButton(self.operatorInterface.defenseStateControl).whileTrue(
            DefenseState(self.drive)
        )

        ModifiableJoystickButton(self.operatorInterface.intakeCoral).whileTrue(
            IntakeCoral(self.intake).repeatedly()
        )
        ModifiableJoystickButton(self.operatorInterface.intakeKnock).whileTrue(
            IntakeKnock(self.intake).repeatedly()
        )
        ModifiableJoystickButton(self.operatorInterface.intakeScoring).whileTrue(
            IntakeScoring(self.intake).repeatedly()
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
