from typing import Optional

# import numpy as np

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Pose3d

import constants
from subsystems.drivesubsystem import DriveSubsystem, VisionObservation
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight
from subsystems.vision.visioniosim import VisionSubsystemIOSim
from util.convenientmath import pose3dFrom2d


class VisionSubsystem(Subsystem):
    camera: VisionSubsystemIO
    visionPose: Optional[VisionObservation]

    def __init__(self, drive: DriveSubsystem) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.poseReceiver = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotPoseArrayKeys.valueKey, Pose2d)
            .subscribe(Pose2d())
        )

        self.visionPosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotVisionPoseArrayKeys.valueKey, Pose2d)
            .publish()
        )
        self.visionPoseValidPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kRobotVisionPoseArrayKeys.validKey)
            .publish()
        )

        self.cameraPosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kCameraLocationPublisherKey, Pose3d)
            .publish()
        )
        self.visionPose = None

        if RobotBase.isReal():
            self.camera = VisionSubsystemIOLimelight()
        else:
            self.camera = VisionSubsystemIOSim()

        self.camera.updateCameraPosition(constants.kRobotToCameraTransform)

    def periodic(self) -> None:
        yaw = self.poseReceiver.get().rotation()
        self.camera.updateRobotYaw(yaw)

        visionPose = self.camera.getRobotFieldPose()

        if visionPose is not None:
            self.cameraPosePublisher.set(
                pose3dFrom2d(visionPose.visionPose) + constants.kRobotToCameraTransform
            )
            self.visionPose = visionPose
            self.visionPosePublisher.set(self.visionPose.visionPose)
            self.visionPoseValidPublisher.set(True)

            self.drive.estimator.addVisionMeasurement(self.visionPose)
        else:
            self.visionPose = None
            self.visionPoseValidPublisher.set(False)
