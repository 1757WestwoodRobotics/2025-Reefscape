from math import hypot
from typing import Optional
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Transform2d, Transform3d, Translation3d
import constants
from subsystems.vision.visionio import VisionSubsystemIO
from util.convenientmath import pose3dFrom2d


class VisionSubsystemIOSim(VisionSubsystemIO):
    def __init__(self) -> None:
        VisionSubsystemIO.__init__(self)
        self.simBotPoseGetter = (
            NetworkTableInstance.getDefault()
            .getDoubleArrayTopic(constants.kSimRobotPoseArrayKey)
            .subscribe([0, 0, 0])
        )
        self.camera = SimCamera(
            "limelight",
            Transform3d(),
            constants.kCameraFOVHorizontal,
            constants.kCameraFOVVertical,
            "ll",
        )
        self.rng = RNG(constants.kSimulationVariation)

    def getRobotFieldPose(self) -> Optional[Pose3d]:
        simPose = Pose2d(*self.simBotPoseGetter.get())
        simPose3d = pose3dFrom2d(simPose)

        seeTag = False
        botPose = Pose3d()
        tagPoses: list[Transform3d] = []

        for tagId, apriltag in constants.kApriltagPositionDict.items():
            if self.camera.canSeeTarget(simPose3d, apriltag):
                rngOffset = Transform3d(
                    Translation3d(
                        self.rng.getNormalRandom(),
                        self.rng.getNormalRandom(),
                        self.rng.getNormalRandom(),
                    ),
                    Rotation3d(),
                )
                botToTagPose = Pose3d() + Transform3d(simPose3d, apriltag)
                botToTagPose = (
                    botToTagPose + rngOffset * botToTagPose.translation().norm()
                )
                tagPoses.append(Transform3d(simPose3d + self.camera.location, apriltag))
                seeTag = True
                botPose = (
                    Pose3d(
                        simPose3d.X(),
                        simPose3d.Y(),
                        simPose3d.Z(),
                        simPose3d.rotation(),
                    )
                    + rngOffset
                )

        return botPose if seeTag else None

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


class SimCamera:
    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        name: str,
        location: Transform3d,
        horizFOV: float,
        vertFOV: float,
        key: str,
    ) -> None:
        self.name = name
        self.location = location
        self.horizFOV = horizFOV
        self.vertFOV = vertFOV
        self.key = key

    def canSeeTarget(self, botPose: Pose3d, targetPose: Pose3d):
        cameraPose = botPose + self.location
        rel = CameraTargetRelation(cameraPose, targetPose)
        return (
            abs(rel.camToTargYaw.degrees()) < self.horizFOV / 2
            and abs(rel.camToTargPitch.degrees()) < self.vertFOV / 2
            and abs(rel.targToCamAngle.degrees()) < 90
        )


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


class VisionSubsystemSim(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.cameras = [
            SimCamera(
                name,
                location,
                constants.kCameraFOVHorizontal,
                constants.kCameraFOVVertical,
                key,
            )
            for name, location, key in zip(
                constants.kPhotonvisionCameraArray,
                constants.kCameraTransformsArray,
                constants.kPhotonvisionKeyArray,
            )
        ]
        self.poseList = []
        self.robotToTags = []

        self.rng = RNG(constants.kSimulationVariation)

    def periodic(self) -> None:
        simPose = Pose2d(*getSDArray(constants.kSimRobotPoseArrayKey, [0, 0, 0]))
        simPose3d = pose3dFrom2d(simPose)

        self.robotToTags = []
        for camera in self.cameras:
            seeTag = False
            botPose = Pose3d()
            tagPoses: list[Transform3d] = []

            for tagId, apriltag in constants.kApriltagPositionDict.items():
                if camera.canSeeTarget(simPose3d, apriltag):
                    rngOffset = Transform3d(
                        Translation3d(
                            self.rng.getNormalRandom(),
                            self.rng.getNormalRandom(),
                            self.rng.getNormalRandom(),
                        ),
                        Rotation3d(),
                    )
                    botToTagPose = Pose3d() + Transform3d(simPose3d, apriltag)
                    botToTagPose = (
                        botToTagPose + rngOffset * botToTagPose.translation().norm()
                    )
                    tagPoses.append(Transform3d(simPose3d + camera.location, apriltag))
                    self.robotToTags.append(
                        (
                            botToTagPose,
                            tagId,
                            botToTagPose.translation().norm()
                            * self.rng.getNormalRandom(),
                        ),
                    )
                    seeTag = True
                    botPose = (
                        Pose3d(
                            simPose3d.X(),
                            simPose3d.Y(),
                            simPose3d.Z(),
                            simPose3d.rotation(),
                        )
                        + rngOffset
                    )

            rel = CameraTargetRelation(simPose3d + camera.location, botPose)
            VisionSubsystemReal.updateAdvantagescopePose(
                botPose + camera.location, camera.key, simPose3d, rel.camToTarg
            )

            if len(tagPoses) == 0:
                continue
            totalDistance = 0
            for transform in tagPoses:
                totalDistance += transform.translation().norm()

            avgDistance = totalDistance / len(tagPoses)

            xyStdDev = (
                constants.kXyStdDevCoeff * (avgDistance * avgDistance) / len(tagPoses)
            )
            thetaStdDev = (
                constants.kThetaStdDevCoeff
                * (avgDistance * avgDistance)
                / len(tagPoses)
            )

            self.poseList.append(
                EstimatedPose(
                    botPose,
                    seeTag,
                    Timer.getFPGATimestamp(),
                    [xyStdDev, xyStdDev, thetaStdDev],
                )
            )

        if len(self.robotToTags) > 0:
            poses, ids, ambiguitys = list(zip(*self.robotToTags))

            poses3d = advantagescopeconvert.convertToSendablePoses(poses)
            putSDArray(constants.kRobotToTagPoseKey, poses3d)
            putSDArray(constants.kRobotToTagIdKey, ids)
            putSDArray(constants.kRobotToTagAmbiguityKey, ambiguitys)


class RNG:
    def __init__(self, stdDev: float) -> None:
        self.stdDev = stdDev
        # self.rng = np.random.normal(0, stdDev, number)
        # self.rngIdx = 0

    def getNormalRandom(self) -> float:
        return sin(1000000 * Timer.getFPGATimestamp()) * self.stdDev
