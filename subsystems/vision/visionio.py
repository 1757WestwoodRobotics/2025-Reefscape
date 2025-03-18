from typing import Optional
from wpimath.geometry import Pose3d, Rotation2d


class VisionSubsystemIO:
    def updateCameraPosition(self, pose: Pose3d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def getRobotFieldPose(self) -> Optional[Pose3d]:
        raise NotImplementedError("Must be implemented by subclass")

    def updateRobotYaw(self, yaw: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")
