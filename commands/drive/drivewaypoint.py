from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.vision.visionsubsystem import VisionSubsystem
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import DriverStation
import constants
from util.convenientmath


class DriveWaypoint(Command):
    def __init__(self, drive: DriveSubsystem, vision: VisionSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.vision = vision

        self.command = Command()

        self.running = False
        self.addRequirements(self.drive, self.vision)

    def initialize(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")
        
    def execute(self) -> None:
        self.command.execute()
        if self.command.isFinished():
            self.command.end(False)
            self.running = False

    def isFinished(self) -> bool:
        return False
    
    def end(self) -> None:
        AutoBuilder._getPose = self.drive.getPose

class DriveLeftReef(DriveWaypoint):
    def __init__(self, drive: DriveSubsystem, vision: VisionSubsystem):
        DriveWaypoint.__init__(self, drive, vision)
    
    def initialize(self):
        targetPose = self.getClosestPose()
        self.running = True
        AutoBuilder._getPose = self.vision.visionPosePublisher.get
        self.command = AutoBuilder.pathfindToPose(
            targetPose, constants.kPathfindingConstraints
        )
        self.command.initialize()
        
    def getClosestPose(self) -> Pose2d:
        currentRotation = self.drive.getRotation()
        targetRotation = Rotation2d.fromDegrees(
            round(currentRotation.degrees() / 60) * 60
        )

        if DriverStation.getAlliance == DriverStation.Alliance.kBlue:
            for position in constants.kLeftReefToOffsetPositionBlue.values():
                if targetRotation.radians() == position.rotation().Z():
                    return position.toPose2d()
        else:
            for position in constants.kLeftReefToOffsetPositionRed.values():
                if targetRotation.radians() == position.rotation().Z():
                    return position.toPose2d()