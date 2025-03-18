from collections import deque
from math import hypot, inf, sin

# import numpy as np

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Pose3d

import constants
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight


class VisionSubsystem(Subsystem):
    camera: VisionSubsystemIO

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.poseReceiver = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotPoseArrayKeys.valueKey, Pose2d)
            .subscribe(Pose2d())
        )

        if RobotBase.isReal():
            self.camera = VisionSubsystemIOLimelight()

    def periodic(self) -> None:
        yaw = self.poseReciever.get().rotation()
        self.camera.updateRobotYaw(yaw)
