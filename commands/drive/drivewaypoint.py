from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import DriverStation
from ntcore import NetworkTableInstance

from subsystems.drivesubsystem import DriveSubsystem
import constants
from util.angleoptimize import optimizeAngle


class DriveWaypoint(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive

        self.command = Command()

        self.running = False
        self.addRequirements(self.drive)

    def initialize(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def execute(self) -> None:
        self.command.execute()
        if self.command.isFinished():
            self.command.end(False)
            self.running = False

    def isFinished(self) -> bool:
        return False

    def end(self, _interrupted: bool) -> None:
        # pylint: disable=W0212
        AutoBuilder._getPose = self.drive.getPose


class DriveLeftReef(DriveWaypoint):
    def __init__(self, drive: DriveSubsystem):
        DriveWaypoint.__init__(self, drive)
        self.visionPoseGetter = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotVisionPoseArrayKeys.valueKey, Pose2d)
            .subscribe(Pose2d())
        )

    def initialize(self):
        targetPose = self.getClosestPose()
        self.running = True
        # pylint: disable=W0212
        AutoBuilder._getPose = self.visionPoseGetter.get
        self.command = AutoBuilder.pathfindToPose(
            targetPose, constants.kPathfindingConstraints
        )
        self.command.initialize()

    def getClosestPose(self) -> Pose2d:
        currentRotation = self.drive.getRotation()
        targetRotation = Rotation2d.fromDegrees(
            round(currentRotation.degrees() / 60) * 60
        )

        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            for position in constants.kLeftReefToOffsetPositionBlue.values():
                if (
                    abs(
                        targetRotation.radians()
                        - optimizeAngle(
                            targetRotation, Rotation2d(position.rotation().Z())
                        ).radians()
                    )
                    <= 0.001
                ):
                    return position.toPose2d()
            return self.drive.getPose()
        else:
            for position in constants.kLeftReefToOffsetPositionRed.values():
                if (
                    abs(
                        targetRotation.radians()
                        - optimizeAngle(
                            targetRotation, Rotation2d(position.rotation().Z())
                        ).radians()
                    )
                    <= 0.001
                ):
                    return position.toPose2d()
            return self.drive.getPose()
