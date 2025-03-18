from typing import Optional
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d
from subsystems.vision.visionio import VisionSubsystemIO


class VisionSubsystemIOLimelight(VisionSubsystemIO):
    def __init__(self) -> None:
        VisionSubsystemIO.__init__(self)
        self.cameraTable = NetworkTableInstance.getDefault().getTable("limelight")
        self.botpose = self.cameraTable.getDoubleArrayTopic(
            "botpose_orb_wpiblue"
        ).subscribe([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.camPoseSetter = self.cameraTable.getDoubleArrayTopic(
            "camerapose_robotspace_set"
        ).publish()
        self.robotOrientationSetter = self.cameraTable.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()

    def getRobotFieldPose(self) -> Optional[Pose3d]:
        (
            poseX,
            poseY,
            poseZ,
            roll,
            pitch,
            yaw,
            latency,
            tagCount,
            tagSpan,
            tagDistAvg,
            tagAreaAvg,
        ) = self.botpose.get()

        return Pose3d(poseX, poseY, poseZ, Rotation3d.fromDegrees(roll, pitch, yaw))


    def updateCameraPosition(self, pose: Pose3d) -> None:
        self.camPoseSetter.set(
            [
                pose.X(),
                pose.Y(),
                pose.Z(),
                pose.rotation().X(),
                pose.rotation().Y(),
                pose.rotation().Z(),
            ]
        )

    def updateRobotYaw(self, yaw: Rotation2d) -> None:
        self.robotOrientationSetter.set([yaw.degrees(), 0, 0, 0, 0, 0])
