from collections import deque
from math import hypot, inf, sin, tan, atan

# import numpy as np

from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import SmartDashboard
from wpilib import RobotBase, Timer
from wpimath.geometry import (
    Transform3d,
    Pose3d,
    Pose2d,
    Rotation2d,
    Translation3d,
    Transform2d,
    Rotation3d,
)

import constants
from util import advantagescopeconvert
from util.convenientmath import pose3dFrom2d
from util.getsdarray import getSDArray, putSDArray


class EstimatedPose:
    def __init__(
        self, pose: Pose3d, hasTargets: bool, timestamp: float, stdDev: list[float]
    ):
        self.pose = pose
        self.hasTargets = hasTargets
        self.timestamp = timestamp
        self.stdDev = stdDev


class VisionCamera:  # hi its landon here
    """
    A unified class for containing information relevant for a vision camera
    for use in a multi-camera vision system
    """

    def __init__(self, camera: PhotonCamera, robotToCameraTransform: Transform3d):
        self.camera = camera
        self.name = camera.getName()

        self.key = f"cameras/{self.name}Camera"

        self.robotToCameraTransform = robotToCameraTransform
        self.cameraToRobotTransform = robotToCameraTransform.inverse()


class VisionSubsystem(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        # self.estimatedPosition = Pose2d()

        # self.camera = PhotonCamera(constants.kPhotonvisionCameraName)

        self.cameras = [
            VisionCamera(PhotonCamera(camera), transform)
            for camera, transform in zip(
                constants.kPhotonvisionCameraArray, constants.kCameraTransformsArray
            )
        ]
        self.field = AprilTagFieldLayout.loadField(AprilTagField.k2024Crescendo)
        self.estimators = [
            PhotonPoseEstimator(
                self.field,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cam.camera,
                cam.robotToCameraTransform,
            )
            for cam in self.cameras
        ]
        self.robotToTags = []

        self.poseList = deque([])

        self.dRobotAngle = Rotation2d()
        # if RobotBase.isSimulation():
        #     inst = NetworkTableInstance.getDefault()
        #     inst.stopServer()
        #     inst.setServer("localhost")
        #     inst.startClient4("Robot Sim")

    def periodic(self) -> None:
        visionPose = getSDArray(constants.kRobotVisionPoseArrayKeys.valueKey, [0, 0, 0])
        robotPose = getSDArray(constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0])

        combinedPose = pose3dFrom2d(Pose2d(visionPose[0], visionPose[1], robotPose[2]))
        self.robotToTags = []
        for camera, estimator in zip(self.cameras, self.estimators):

            camEstPose = estimator.update()
            camResult = camera.camera.getLatestResult()
            if camEstPose:
                # only send stuff is pose is detected
                botPose = camEstPose.estimatedPose
                timestamp = camEstPose.timestampSeconds
                tagsUsed = camEstPose.targetsUsed

                isMultitag = len(tagsUsed) > 1

                if isMultitag:
                    avgDistance = botPose.transformBy(
                        camera.robotToCameraTransform
                    ).relativeTo(estimator._fieldTags.getOrigin()).translation().norm()
                else:
                    bestTarget = camResult.getBestTarget()
                    assert bestTarget is not None
                    avgDistance = bestTarget.getBestCameraToTarget().translation().norm()


                VisionSubsystem.updateAdvantagescopePose(
                    botPose, camera.key, combinedPose, camera.cameraToRobotTransform
                )

                xyStdDev = (
                    constants.kXyStdDevCoeff
                    * (avgDistance * avgDistance)
                    / len(tagsUsed)
                )
                thetaStdDev = (
                    (
                        constants.kThetaStdDevCoeff
                        * (avgDistance * avgDistance)
                        / len(tagsUsed)
                    )
                    if isMultitag
                    else inf
                )

                self.poseList.append(
                    EstimatedPose(
                        botPose,
                        camResult.hasTargets(),
                        timestamp,
                        [xyStdDev, xyStdDev, thetaStdDev],
                    )
                )

        if len(self.robotToTags) > 0:
            poses, ids, ambiguitys = list(zip(*self.robotToTags))

            poses3d = advantagescopeconvert.convertToSendablePoses(poses)
            putSDArray(constants.kRobotToTagPoseKey, poses3d)
            putSDArray(constants.kRobotToTagIdKey, ids)
            putSDArray(constants.kRobotToTagAmbiguityKey, ambiguitys)

    @staticmethod
    def updateAdvantagescopePose(
        cameraPose3d: Pose3d,
        cameraKey: str,
        botPose: Pose3d,
        cameraToRobotTransform: Transform3d,
    ) -> None:
        cameraPose = advantagescopeconvert.convertToSendablePoses(
            [cameraPose3d, botPose + cameraToRobotTransform.inverse()]
        )
        putSDArray(cameraKey, cameraPose)


class CameraTargetRelation:
    def __init__(self, cameraPose: Pose3d, targetPose: Pose3d) -> None:
        self.cameraPose = cameraPose
        self.camToTarg = Transform3d(cameraPose, targetPose)
        self.camToTargDist = self.camToTarg.translation().norm()
        self.camToTargDistXY = hypot(
            self.camToTarg.translation().X(), self.camToTarg.translation().Y()
        )
        self.camToTargYaw = Rotation2d(self.camToTarg.X(), self.camToTarg.Y())
        self.camToTargPitch = Rotation2d(self.camToTargDistXY, -self.camToTarg.Z())
        self.camToTargAngle = Rotation2d(
            hypot(self.camToTargYaw.radians(), self.camToTargPitch.radians())
        )

        self.targToCam = Transform3d(targetPose, cameraPose)
        self.targToCamYaw = Rotation2d(self.targToCam.X(), self.targToCam.Y())
        self.targToCamPitch = Rotation2d(self.camToTargDistXY, -self.targToCam.Z())
        self.targToCamAngle = Rotation2d(
            hypot(self.targToCamYaw.radians(), self.targToCamPitch.radians())
        )


class RNG:
    def __init__(self, stdDev: float) -> None:
        self.stdDev = stdDev
        # self.rng = np.random.normal(0, stdDev, number)
        # self.rngIdx = 0

    def getNormalRandom(self) -> float:
        return sin(1000000 * Timer.getFPGATimestamp()) * self.stdDev
        # self.rngIdx = (self.rngIdx + 1) % self.number
        # return self.rng[self.rngIdx]
