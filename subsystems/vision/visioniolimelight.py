from typing import Optional
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d, Pose2d
from subsystems.vision.visionio import VisionSubsystemIO

from util.convenientmath import clamp
import constants


class VisionSubsystemIOLimelight(VisionSubsystemIO):
    def __init__(self) -> None:
        VisionSubsystemIO.__init__(self)
        self.cameraTable = NetworkTableInstance.getDefault().getTable("limelight")
        self.botpose = self.cameraTable.getDoubleArrayTopic(
            "botpose_orb_wpiblue"
        ).subscribe([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.camPoseSetter = self.cameraTable.getDoubleArrayTopic(
            "camerapose_robotspace_set"
        ).publish()
        self.robotOrientationSetter = self.cameraTable.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()
        self.robotPoseGetter = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotPoseArrayKeys.valueKey, Pose2d)
            .subscribe(Pose2d())
        )

    def getRobotFieldPose(self) -> Optional[Pose3d]:
        botPose = self.botpose.get()
        poseX, poseY, poseZ = botPose[0:3]
        rotation = self.robotPoseGetter.get().rotation().radians()
        return Pose3d(
            clamp(poseX, 0, constants.kFieldLength),
            clamp(poseY, 0, constants.kFieldWidth),
            poseZ,
            Rotation3d(0, 0, rotation),
        )

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self.camPoseSetter.set(
            [
                transform.X(),
                transform.Y(),
                transform.Z(),
                transform.rotation().X() / constants.kRadiansPerDegree,
                transform.rotation().Y() / constants.kRadiansPerDegree,
                transform.rotation().Z() / constants.kRadiansPerDegree,
            ]
        )

    def updateRobotYaw(self, yaw: Rotation2d) -> None:
        self.robotOrientationSetter.set([yaw.degrees(), 0, 0, 0, 0, 0])
