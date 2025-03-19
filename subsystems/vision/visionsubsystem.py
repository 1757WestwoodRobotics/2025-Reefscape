from typing import Optional

# import numpy as np

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import RobotBase
from wpimath.geometry import Pose2d

import constants
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight
from subsystems.vision.visioniosim import VisionSubsystemIOSim


class VisionSubsystem(Subsystem):
    camera: VisionSubsystemIO
    visionPose: Optional[Pose2d]

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

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
        self.visionPose = None

        if RobotBase.isReal():
            self.camera = VisionSubsystemIOLimelight()
        else:
            self.camera = VisionSubsystemIOSim()

    def periodic(self) -> None:
        yaw = self.poseReceiver.get().rotation()
        self.camera.updateRobotYaw(yaw)

        visionPose = self.camera.getRobotFieldPose()

        if visionPose is not None:
            self.visionPose = visionPose.toPose2d()
            self.visionPosePublisher.set(self.visionPose)
            self.visionPoseValidPublisher.set(True)
        else:
            self.visionPose = None
            self.visionPoseValidPublisher.set(False)
