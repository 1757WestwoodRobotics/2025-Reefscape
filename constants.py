# pylint:disable=too-many-lines
"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!

Physical constants must have their units specified
Default units:
    Length: meters
    Angle: radians

Axes Convention (right hand rule):
    Translation:
        +X: forward
        +Y: left
        +Z: up

    Rotation:
        +rotate around X: counterclockwise looking from the front, 0 aligned with +Y
        +rotate around Y: counterclockwise looking from the left, 0 aligned with +Z
        +rotate around Z: counterclockwise looking from the top, 0 aligned with +X

Swerve Module Layout:
    Drive (input) -> Drive Gearing -> Wheel (output)
    Steer (input) -> Steer Gearing -> Swerve (output)
"""

import math
from phoenix6.configs.config_groups import CurrentLimitsConfigs
from wpimath.geometry import (
    Pose3d,
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Transform2d,
    Translation2d,
)
from wpimath.system.plant import DCMotor
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    PIDConstants,
    ReplanningConfig,
)

from pathplannerlib.auto import PathConstraints

from util.keyorganization import OptionalValueKeys

# Basic units
kInchesPerFoot = 12
"""inches / foot"""

kCentimetersPerInch = 2.54
"""centimeters / inch"""

kCentimetersPerMeter = 100
"""centimeters / meter"""

kMetersPerInch = kCentimetersPerInch / kCentimetersPerMeter
"""meters / inch"""

kMetersPerFoot = kMetersPerInch * kInchesPerFoot
"""meters / foot"""

kRadiansPerRevolution = 2 * math.pi
"""radians / revolution"""

kDegeersPerRevolution = 360
"""degrees / revolution"""

kRadiansPerDegree = kRadiansPerRevolution / kDegeersPerRevolution
"""radians / degree"""

kMillisecondsPerSecond = 1000 / 1
"""milliseconds / second"""

kSecondsPerMinute = 60 / 1
"""seconds / minute"""

kRPMPerAngularVelocity = (1 / kRadiansPerRevolution) * kSecondsPerMinute
"""RPM / (radians / second)"""

kGravity = 9.802  # new york gravity
"""m / s / s"""

# Debug parameters
kPrintFrequency = 2
""" 1 / second"""

kPrintPeriod = 1 / kPrintFrequency
"""seconds"""

# Field Physical parameters
kFieldLength = 54 * kMetersPerFoot + 3.25 * kMetersPerInch
"""meters"""

kFieldWidth = 26 * kMetersPerFoot + 3.5 * kMetersPerInch
"""meters"""

# Robot Physical parameters
kRobotWidth = 28 * kMetersPerInch
"""meters"""

kRobotLength = 26 * kMetersPerInch
"""meters"""

kSwerveModuleCenterToRobotCenterWidth = 10.375 * kMetersPerInch
"""meters"""
kSwerveModuleCenterToRobotCenterLength = 9.375 * kMetersPerInch
"""meters"""

kSwerveModuleDistanceFromRobotCenter = pow(
    pow(kSwerveModuleCenterToRobotCenterWidth, 2)
    + pow(kSwerveModuleCenterToRobotCenterLength, 2),
    0.5,
)
"""meters (c = (a^2 + b^2) ^ 0.5)"""

kFrontLeftWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kFrontRightWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackLeftWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackRightWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kWheelDiameter = 4 * kMetersPerInch
"""meters"""

kWheelRadius = kWheelDiameter / 2
"""meters"""

kWheelCircumference = kWheelRadius * 2 * math.pi
"""meters"""

kWheelDistancePerRevolution = kWheelCircumference
"""meters / revolution"""

kWheelDistancePerRadian = kWheelDistancePerRevolution / kRadiansPerRevolution
"""meters / radian"""

kDriveGearingRatio = (50 / 14) * (16 / 28) * (45 / 15)
"""dimensionless"""

kSteerGearingRatio = 150 / 7
"""dimensionless"""

kMaxMotorAngularVelocity = DCMotor.krakenX60().freeSpeed
"""radians / second"""

kMaxWheelAngularVelocity = kMaxMotorAngularVelocity / kDriveGearingRatio
"""radians / second"""

kMaxWheelLinearVelocity = kWheelDistancePerRadian * kMaxWheelAngularVelocity
"""meters / second"""

kMinWheelLinearVelocity = 0.002
"""meters / second"""

kMaxSteerAngularVelocity = kMaxMotorAngularVelocity / kSteerGearingRatio
"""radians / second"""

kMaxForwardLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxSidewaysLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxRotationAngularVelocity = (
    kMaxWheelLinearVelocity / kSwerveModuleDistanceFromRobotCenter
)
"""radians / second (omega = v / r)"""

kMaxWheelLinearAcceleration = kMaxWheelLinearVelocity / 1
"""meters / second^2"""

kMaxForwardLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxSidewaysLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxRotationAngularAcceleration = kMaxRotationAngularVelocity / 0.5
"""radians / second^2"""

kFrontLeftModuleName = "front_left"
kFrontRightModuleName = "front_right"
kBackLeftModuleName = "back_left"
kBackRightModuleName = "back_right"

kKilogramToLbs = 0.454

# Limelight
kLimelightTargetInvalidValue = 0.0
kLimelightTargetValidValue = 1.0
kLimelightMinHorizontalFoV = Rotation2d.fromDegrees(-29.8)
kLimelightMaxHorizontalFoV = Rotation2d.fromDegrees(29.8)
kLimelightMinVerticalFoV = Rotation2d.fromDegrees(-22.85)
kLimelightMaxVerticalFoV = Rotation2d.fromDegrees(22.85)
kLimelightNetworkTableName = "limelight"
kLimelightTargetValidKey = "tv"
kLimelightTargetHorizontalAngleKey = "tx"
kLimelightTargetVerticalAngleKey = "ty"
kLimelightLEDModeKey = "ledMode"
kLimelightTrackerModuleName = "limelight"
kLimelightRelativeToRobotTransform = Transform3d(
    Pose3d(),
    Pose3d(0.236, 0.206, 0.197, Rotation3d()),
)

# Photonvision related
kPhotonvisionCameraName = "camcam"
kPhotonvisionCameraArray = ["frontLeft", "frontRight", "backLeft", "backRight"]

kPhotonvisionFrontLeftCameraKey = "cameras/frontLeftCamera"
kPhotonvisionFrontRightCameraKey = "cameras/frontRightCamera"
kPhotonvisionBackLeftCameraKey = "cameras/backLeftCamera"
kPhotonvisionBackRightCameraKey = "cameras/backRightCamera"

kPhotonvisionKeyArray = [
    kPhotonvisionFrontLeftCameraKey,
    kPhotonvisionFrontRightCameraKey,
    kPhotonvisionBackLeftCameraKey,
    kPhotonvisionBackRightCameraKey,
]

kPhotonvisionNoteCameraKey = "noteCamera"
kNoteInViewKey = OptionalValueKeys("noteInView")

kNoteCameraPitch = 30 * kRadiansPerDegree  # below horizontal
kNoteCameraYaw = 20 * kRadiansPerDegree
kRobotToNoteCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(
        0.330296, -0.333443, 0.570646, Rotation3d(0, kNoteCameraPitch, kNoteCameraYaw)
    ),
)

kRobotToNoteCameraTransformNoPitch = Transform3d(
    Pose3d(),
    Pose3d(0.330296, -0.333443, 0.570646, Rotation3d(0, 0, kNoteCameraYaw)),
)

kCameraFOVHorizontal = 75.9  # degrees
kCameraFOVVertical = 47.4  # degrees

kSimulationVariation = 0.001  # meters, as a standard deviation


kRobotToFrontLeftCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(
        11.486 * kMetersPerInch,
        10.991 * kMetersPerInch,
        8.475 * kMetersPerInch,
        Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0).rotateBy(
            Rotation3d(0.0, 0.0, 15.0 * kRadiansPerDegree)
        ),
    ),
)
kRobotToFrontRightCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(
        11.306 * kMetersPerInch,
        -12.749 * kMetersPerInch,
        9.238 * kMetersPerInch,
        Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0),
    ),
)
kRobotToBackLeftCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(
        -11.486 * kMetersPerInch,
        10.990 * kMetersPerInch,
        8.475 * kMetersPerInch,
        Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0).rotateBy(
            Rotation3d(0.0, 0.0, (180 - 15.0) * kRadiansPerDegree)
        ),
    ),
)
kRobotToBackRightCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(
        -11.486 * kMetersPerInch,
        -10.991 * kMetersPerInch,
        8.475 * kMetersPerInch,
        Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0).rotateBy(
            Rotation3d(0.0, 0.0, (180 + 15.0) * kRadiansPerDegree)
        ),
    ),
)
kCameraTransformsArray = [
    kRobotToFrontLeftCameraTransform,
    kRobotToFrontRightCameraTransform,
    kRobotToBackLeftCameraTransform,
    kRobotToBackRightCameraTransform,
]

# CANivore
kCANivoreName = "canivore"

# Motors
kFrontLeftDriveMotorId = 10
kFrontLeftSteerMotorId = 11
kFrontRightDriveMotorId = 12
kFrontRightSteerMotorId = 13
kBackLeftDriveMotorId = 14
kBackLeftSteerMotorId = 15
kBackRightDriveMotorId = 16
kBackRightSteerMotorId = 17

kDriveCurrentLimit = (
    CurrentLimitsConfigs()
    .with_stator_current_limit(35)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(35)
    .with_supply_current_limit_enable(True)
)

kDriveAngularVelocityCoeff = 0.01  # while translating and rotating, need a bit extra motion to compensate for moving reference frame

# Pigeon
kPigeonCANId = 44

# Encoders
kFrontLeftSteerEncoderId = 40
kFrontRightSteerEncoderId = 41
kBackLeftSteerEncoderId = 42
kBackRightSteerEncoderId = 43

kCANcoderPulsesPerRevolution = 4096
"""pulses / revolution"""

kCANcoderPulsesPerRadian = kCANcoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kTalonEncoderPulsesPerRevolution = 2048
"""pulses / revolution"""

kTalonEncoderPulsesPerRadian = kTalonEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kDriveEncoderPulsesPerRevolution = kTalonEncoderPulsesPerRevolution
"""pulses / revolution"""

kDriveEncoderPulsesPerRadian = kDriveEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kDriveEncoderPulsesPerMeter = kDriveEncoderPulsesPerRadian / kWheelDistancePerRadian
"""pulses / meter"""

kWheelEncoderPulsesPerRevolution = kDriveEncoderPulsesPerRevolution * kDriveGearingRatio
"""pulses / revolution"""

kWheelEncoderPulsesPerRadian = kWheelEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kWheelEncoderPulsesPerMeter = kWheelEncoderPulsesPerRadian / kWheelDistancePerRadian
"""pulses / meter"""

kSteerEncoderPulsesPerRevolution = kTalonEncoderPulsesPerRevolution
"""pulses / revolution"""

kSteerEncoderPulsesPerRadian = kSteerEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kSwerveEncoderPulsesPerRevolution = (
    kSteerEncoderPulsesPerRevolution * kSteerGearingRatio
)
"""pulses / revolution"""

kSwerveEncoderPulsesPerRadian = (
    kSwerveEncoderPulsesPerRevolution / kRadiansPerRevolution
)
"""pulses / radian"""

# CTRE
k100MillisecondsPerSecond = 10 / 1  # there are 10 groups of 100 milliseconds per second
"""100 milliseconds / second
   CTRE reports velocities in units of (quantity / 100 milliseconds)
   This factor is used to convert to (quantity / 1 second)
"""

kTalonVelocityPerRPM = (
    kTalonEncoderPulsesPerRevolution / kSecondsPerMinute
) / k100MillisecondsPerSecond
"""(pulses / 100 milliseconds) / RPM"""


kTalonVelocityPerAngularVelocity = kTalonVelocityPerRPM * kRPMPerAngularVelocity
"""(pulses / 100 milliseconds) / (radians / second)"""

kConfigurationTimeoutLimit = int(5 * kMillisecondsPerSecond)
"""milliseconds"""

kDrivePIDSlot = 0
kDrivePGain = 0.001
kDriveIGain = 0.0
kDriveDGain = 0.0
kDriveVGain = 0.01

kSteerPIDSlot = 0
kSteerPGain = 2
kSteerIGain = 0.0
kSteerDGain = 0

kFrontLeftDriveInverted = True
kFrontRightDriveInverted = False
kBackLeftDriveInverted = True
kBackRightDriveInverted = False

kFrontLeftSteerInverted = False
kFrontRightSteerInverted = False
kBackLeftSteerInverted = False
kBackRightSteerInverted = False

"""
To determine encoder offsets (with robot ON and DISABLED):
  1. Rotate all swerve modules so that the wheels:
     * are running in the forwards-backwards direction
     * have the wheel bevel gears facing inwards towards the
       center-line of the robot
  2. Run Phoenix Tuner
  3. Select desired encoder
  4. Go to "Config" tab
  5. Click "Factory Default"
  6. Go to "Self-Test Snapshot" tab
  7. Click "Self-Test Snapshot"
  8. Record value from line: "Absolute Position (unsigned):"
"""
kFrontLeftAbsoluteEncoderOffset = 0.417969
"""rotations"""

kFrontRightAbsoluteEncoderOffset = -0.055176
"""rotations"""

kBackLeftAbsoluteEncoderOffset = 0.452637
"""rotations"""

kBackRightAbsoluteEncoderOffset = 0.105225
"""rotations"""

kRobotPoseArrayKeys = OptionalValueKeys("RobotOdometryPose")

kRobotVisionPoseWeight = 0.00  # 5% vision data

kDriveVelocityKeys = "robotVelocity"
kDriveAccelLimit = 7
kRobotUpdatePeriod = 1 / 50
"""seconds"""
kLimelightUpdatePeriod = 1 / 10
"""seconds"""

# Vision parameters
kTargetAngleRelativeToRobotKeys = OptionalValueKeys("TargetAngleRelativeToRobot")
kTargetDistanceRelativeToRobotKeys = OptionalValueKeys("TargetDistanceRelativeToRobot")
kTargetFacingAngleRelativeToRobotKeys = OptionalValueKeys(
    "TargetFacingAngleRelativeToRobot"
)
kTargetPoseArrayKeys = OptionalValueKeys("TargetPoseArray")
kRobotVisionPoseArrayKeys = OptionalValueKeys("EstimatedRobotPose")
kRobotToTagPoseKey = "vision/poses"
kRobotToTagIdKey = "vision/ids"
kRobotToTagAmbiguityKey = "vision/ambiguity"

kTargetName = "Target"

kApriltagPositionDict = {  # thanks 6328 for FieldConstants!
    1: Pose3d(
        (kMetersPerInch * 593.68),
        (kMetersPerInch * 9.68),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    2: Pose3d(
        (kMetersPerInch * 637.21),
        (kMetersPerInch * 34.79),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    3: Pose3d(
        (kMetersPerInch * 652.73),
        (kMetersPerInch * 196.17),
        (kMetersPerInch * 57.13),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    4: Pose3d(
        (kMetersPerInch * 652.73),
        (kMetersPerInch * 218.42),
        (kMetersPerInch * 57.13),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    5: Pose3d(
        (kMetersPerInch * 578.77),
        (kMetersPerInch * 323.00),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    6: Pose3d(
        (kMetersPerInch * 72.5),
        (kMetersPerInch * 323.00),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    7: Pose3d(
        (kMetersPerInch * -1.50),
        (kMetersPerInch * 218.42),
        (kMetersPerInch * 57.13),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
    ),
    8: Pose3d(
        (kMetersPerInch * -1.50),
        (kMetersPerInch * 196.17),
        (kMetersPerInch * 57.13),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
    ),
    9: Pose3d(
        (kMetersPerInch * 14.02),
        (kMetersPerInch * 34.79),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
    ),
    10: Pose3d(
        (kMetersPerInch * 57.54),
        (kMetersPerInch * 9.68),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
    ),
    # 11: Pose3d(
    #     (kMetersPerInch * 468.69),
    #     (kMetersPerInch * 146.19),
    #     (kMetersPerInch * 52.00),
    #     Rotation3d(0.0, 0.0, kRadiansPerDegree * 300),
    # ),
    # 12: Pose3d(
    #     (kMetersPerInch * 468.69),
    #     (kMetersPerInch * 177.10),
    #     (kMetersPerInch * 52.00),
    #     Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
    # ),
    # 13: Pose3d(
    #     (kMetersPerInch * 441.74),
    #     (kMetersPerInch * 161.62),
    #     (kMetersPerInch * 52.00),
    #     Rotation3d(0.0, 0.0, kRadiansPerDegree * 180),
    # ),
    # 14: Pose3d(
    #     (kMetersPerInch * 209.48),
    #     (kMetersPerInch * 161.62),
    #     (kMetersPerInch * 52.00),
    #     Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
    # ),
    # 15: Pose3d(
    #     (kMetersPerInch * 182.73),
    #     (kMetersPerInch * 177.10),
    #     (kMetersPerInch * 52.00),
    #     Rotation3d(0.0, 0.0, kRadiansPerDegree * 120),
    # ),
    # 16: Pose3d(
    #     (kMetersPerInch * 182.73),
    #     (kMetersPerInch * 146.19),
    #     (kMetersPerInch * 52.00),
    #     Rotation3d(0.0, 0.0, kRadiansPerDegree * 240),
    # ),
}

# Autonomous
kAutoDriveDistance = -8 * kWheelCircumference
"""meters"""

kAutoFrontwaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoSidewaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoDistanceThreshold = 6 * kMetersPerInch
"""meters"""

kAutoDriveSpeedFactor = 0.5
"""dimensionless"""

kAutoWaitDuration = 1
"""seconds"""

kAutoTargetOffset = Translation2d(2, 0)
"""[meters, meters]"""

kAutoDuration = 15
"""seconds"""

# Target relative drive
kTargetRelativeDriveAnglePGain = 1
kTargetRelativeDriveAngleIGain = 0
kTargetRelativeDriveAngleDGain = 0

kRotationPGain = 0.1
kRotationIGain = 0
kRotationDGain = 0.00

# Drive to Target
kDriveToTargetDistancePGain = 0.5
kDriveToTargetDistanceIGain = 0
kDriveToTargetDistanceDGain = 0

kDriveToTargetAnglePGain = 0.5
kDriveToTargetAngleIGain = 0
kDriveToTargetAngleDGain = 0

kDriveToTargetDistanceTolerance = 10 / kCentimetersPerMeter
"""meters"""

kDriveToTargetLinearVelocityTolerance = 1 / kCentimetersPerMeter / 1
"""meters / second"""

kDriveToTargetAngleTolerance = 5 * kRadiansPerDegree
"""radians"""

kDriveToTargetAngularVelocityTolerance = 5 * kRadiansPerDegree / 1
"""radians / second"""

# Trajectory Following
kTrajectoryPositionPGain = 3
kTrajectoryPositionIGain = 0
kTrajectoryPositionDGain = 0

kTrajectoryAnglePGain = 7
kTrajectoryAngleIGain = 0
kTrajectoryAngleDGain = 0

kPathFollowingConfig = HolonomicPathFollowerConfig(
    PIDConstants(
        kTrajectoryPositionPGain, kTrajectoryPositionIGain, kTrajectoryPositionDGain
    ),
    PIDConstants(kTrajectoryAnglePGain, kTrajectoryAngleIGain, kTrajectoryAngleDGain),
    kMaxForwardLinearVelocity / 2,
    kFrontLeftWheelPosition.norm(),
    ReplanningConfig(),
)

# Operator Interface
kXboxJoystickDeadband = 0.1
"""dimensionless"""

kKeyboardJoystickDeadband = 0.0
"""dimensionless"""

kControllerMappingFilename = "ControlScheme.json"

kChassisRotationXAxisName = "chassisXRotation"
kChassisRotationYAxisName = "chassisYRotation"
kChassisForwardsBackwardsAxisName = "chassisForwardsBackwards"
kChassisSideToSideAxisName = "chassisSideToSide"

kFieldRelativeCoordinateModeControlButtonName = "fieldRelativeCoordinateModeControl"
kResetGyroButtonName = "resetGyro"
kTargetRelativeCoordinateModeControlButtonName = "targetRelativeCoordinateModeControl"
kDriveToTargetControlButtonName = "driveToTargetControl"
kXboxTriggerActivationThreshold = 0.5

kTurboSpeedButtonName = "turboSpeed"
kNormalSpeedMultiplier = 0.50  # half full on normal
kTurboSpeedMultiplier = 0.95  # full speed!!!

# Simulation Parameters
kSimulationRotationalInertia = 0.0002
kSimulationRotationalInertiaFlywheel = 0.002
kSimMotorResistance = 0.002
"""[meters, meters, radians]"""

kSimDefaultRobotLocation = Pose2d(0, 0, 0)
kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in

kSimRobotPoseArrayKey = "SimRobotPoseArray"
kSimRobotVelocityArrayKey = "SimRobotVelocityArray"

"""meters"""

kMotorBaseKey = "motors"

# waypoint setter constraints
kMaxWaypointTranslationalVelocity = kMaxForwardLinearVelocity
kMaxWaypointTranslationalAcceleration = kMaxWaypointTranslationalVelocity * 3

kPossibleWaypoints = []
kWaypointJoystickVariation = 0.1
"""meters"""

kTargetWaypointPoseKey = "waypoint/target"
kTargetWaypointXControllerKey = "waypoint/x"
kTargetWaypointYControllerKey = "waypoint/y"
kTargetWaypointThetaControllerKey = "waypoint/theta"

# lights
kCANdleID = 2

# Field

kSpeakerCenterBlue = Pose3d(0, 5.549, 2.12, Rotation3d())
kSpeakerCenterRed = Pose3d(kFieldLength, 5.549, 2.12, Rotation3d())

kNotesStartingMidline = [
    Pose3d(8.258, 7.462, 0.03018, Rotation3d()),
    Pose3d(8.258, 5.785, 0.03018, Rotation3d()),
    Pose3d(8.258, 4.109, 0.03018, Rotation3d()),
    Pose3d(8.258, 2.432, 0.03018, Rotation3d()),
    Pose3d(8.258, 0.756, 0.03018, Rotation3d()),
]

kNotesStartingBlueWing = [
    Pose3d(2.884, 4.109, 0.03018, Rotation3d()),
    Pose3d(2.884, 5.557, 0.03018, Rotation3d()),
    Pose3d(2.884, 7.004, 0.03018, Rotation3d()),
]

kNotesStartingRedWing = [
    Pose3d(13.63, 4.109, 0.03018, Rotation3d()),
    Pose3d(13.63, 5.557, 0.03018, Rotation3d()),
    Pose3d(13.63, 7.004, 0.03018, Rotation3d()),
]

kNoteLoadingStationPositionBlue = Pose3d(15, 1, 0, Rotation3d())
kNoteLoadingStationPositionRed = Pose3d(54 * kMetersPerFoot - 15, 1, 0, Rotation3d())

kSimNotePositionsKey = "SimNotesPositions"

# Logging
kSwerveActualStatesKey = "swerve/actual"
kSwerveExpectedStatesKey = "swerve/expected"
kConsoleLog = "log"
kPDHCanID = 1
kPDHPublishKey = "powerDistribution"

kJoystickKeyLogPrefix = "DriverStation"
kFieldSimTargetKey = "SimTargets"
kFieldRelativeTargets = "RelTargets"

# Velocity Dynamic Control
kVelocitySetpoint1ControlKey = "controls/velocity/Setpoint 1"
kVelocitySetpoint2ControlKey = "controls/velocity/Setpoint 2"
kVelocityControlGearRatio = "controls/velocity/ratio"

kVelocityControlCANId = 3
kVelocityControlPGain = 0.001
kVelocityControlIGain = 0
kVelocityControlDGain = 0

kVelocityControlMotorType = DCMotor.falcon500()
kVelocityControlkV = 0.01

# Intake Mechanism, need to replace values
kIntakeCANID = 25
kIntakeName = "IntakeMotor"
kIntakePIDSlot = 0
kIntakePGain = 0.8
kIntakeIGain = 0
kIntakeDGain = 0
kIntakeKv = 0.00200  # stolen from shooter :)
kIntakeKs = 0.33329

kPivotCANID = 19
kPivotName = "PivotMotor"
kPivotPGain = 0.9
kPivotIGain = 0
kPivotDGain = 0

kPivotAccel = 400
kPivotVel = 150

kPivotGearRatio = (4 / 1) * (50 / 16) * (84 / 16)

kIntakeInverted = True
kPivotInverted = False

kPivotEncoderID = 46
kPivotEncoderOffset = 0.363525 - 0.25  # revolutions, get from phoenix tuner

kIntakeStateKey = "intake/state"
kIntakeHasNoteKey = "intake/hasNote"
kIntakeAtPositionKey = "intake/atPosition"
kIntakePoseKey = "intake/pose"
kIntakeFrontSwitchKey = "intake/limits/front"
kIntakeBackSwitchKey = "intake/limits/back"
kIntakeCanMoveKey = "intake/debug/canMove"
kIntakeHoldSetKey = "intake/debug/holdSet"
kIntakePutInPlaceKey = "intake/debug/putInPlace"
kIntakeSubsystemKey = "intake/subsystem"

kIntakeIntakingVoltage = "intake/intakingVoltage"
kIntakeFineVoltage = "intake/fineVoltage"

# relative to horizontal
kHandoffAngle = Rotation2d.fromDegrees(-4.3)  # we should change this bc it's too far
kFeedAngle = Rotation2d(0.061)  # we should change this bc it's too far
kFloorPositionAngle = Rotation2d.fromDegrees(217.28378)
kStagingPositionAngle = Rotation2d.fromDegrees(
    75
)  # Safe place roughly vertical before going to amp or trap
kAmpScoringPositionAngle = Rotation2d.fromDegrees(90)
kTrapPositionAngle = Rotation2d.fromDegrees(100)

kIntakeArmLength = 0.251
kIntakePivotTolerance = 0.1  # radians
# Percent Voltage
kIntakePercentageVoltage = 0.35
kIntakeFineControlVoltage = 0.07
kIntakeFineVelocityRPM = 250
# EncoderTicks, to be changed
kIntakeSafetyPositionOffset = 1.5 * 3
kIntakeFirstSensorPositionOffset = 2 * 3
kIntakePositionThreshold = 0.25
kIntakeStoppedThreshold = 10  # rpm

kPivotAngleKey = "intake/pivotAngle"
kIntakeSpeedKey = "intake/speed"

kAngleMotorRatio = (64 / 12) * (64 / 16) * (60 / 18)
kShootingMotorRatio = 24 / 36

# change numbers later

kAngleMotorCANId = 18
kAngleMotorName = "ShooterAngleMotor"
kAngleMotorPGain = 0.9
kAngleMotorIGain = 0
kAngleMotorDGain = 0
kAngleMotorInverted = True

kAngleMotorAccel = 300
kAngleMotorVel = 100

# gains taken from 6328 comp https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/flywheels/FlywheelConstants.java#L20

kLeftShootingMotorCANId = 20
kLeftShootingMotorName = "LeftShootingMotor"
kLeftShootingMotorPIDSlot = 0
kLeftShootingMotorPGain = 0.11
kLeftShootingMotorIGain = 0
kLeftShootingMotorDGain = 0
kLeftShootingMotorInverted = False
kLeftShootingMotorKv = 0.004  # V / rpm
kLeftShootingMotorKs = 0.28367

# Kv taken from motor specifications

kRightShootingMotorCANId = 21
kRightShootingMotorName = "RightShootingMotor"
kRightShootingMotorPIDSlot = 0
kRightShootingMotorPGain = 0.12
kRightShootingMotorIGain = 0
kRightShootingMotorDGain = 0
kRightShootingMotorInverted = False
kRightShootingMotorKv = 0.004  # V / rpm

kAngleMotorMappingFunction = lambda x, y: x * y
kLeftShootingMotorMappingFunction = lambda x, y: x * y
kRightShootingMotorMappingFunction = lambda x, y: x * y
kRobotAngleMappingFunction = lambda x, y: x * y

kShooterAngleEncoderCANId = 45
kShooterAngleEncoderOffset = 0.154785 - 0.25  # revolutions

# radians
kShooterAngleKey = "shooter/angle"
kShooterAngleMotorKey = "shooter/angleMotor"
kShooterPosesKey = "shooter/poses"
kLeftShootingMotorSpeedKey = "shooter/leftMotorSpeed"
kRightShootingMotorSpeedKey = "shooter/rightMotorSpeed"
kLeftShootingMotorTargetKey = "shooter/leftTarget"
kRightShootingMotorTargetKey = "shooter/rightTarget"
kShooterPivotTargetKey = "shooter/pivotTarget"
kShooterSubsystemKey = "shooter/subsystem"

kShooterAngleTolerance = Rotation2d(0.05)
# in RPS
kShooterSpeedTolerance = 3

kShooterAngleOnTargetKey = "shooter/angleOnTarget"
kLeftShootingMotorOnTargetKey = "shooter/leftMotorOnTarget"
kRightShootingMotorOnTargetKey = "shooter/rightMotorOnTarget"
kRobotAngleOnTargetKey = "shooter/robotAngleOnTarget"
kReadyToShoot = "shooter/ready"

# from horizontal
kShooterMaxAngle = Rotation2d.fromDegrees(64.028164)
kShooterMinAngle = Rotation2d.fromDegrees(10.207848)

kShootingMotorCurrentLimit = 80
kAngleMotorCurrentLimit = 80

kShooterSubwooferAngle = Rotation2d(0.9)
kShooterSubwooferSpeed = 5000  # NEEDS UPDATE

kPodiumShooterAngle = Rotation2d(0.639)  # NEEDS UPDATE
kPodiumShooterSpeed = 4000  # NEEDS UPDATE

kShooterPassAngle = kShooterMinAngle
kShooterPassSpeed = 4000

kShooterIdleSpeed = 0

kShooterManualModeKey = "shooter/manualMode"
kShooterAngleFudgeKey = "shooter/fudge/angle"
kLeftMotorFudgeKey = "shooter/fudge/leftMotor"
kRightMotorFudgeKey = "shooter/fudge/rightMotor"

kShooterFudgeGlobalRed = Rotation2d(0.025)
kShooterFudgeGlobalBlue = Rotation2d(0)


kShooterCalcSpeed = "shooter/calculated/speed"
kShooterCalcAngle = "shooter/calculated/angle"
kShooterCalcDistance = "shooter/calculated/distance"

kShootingMotorFudgeAmount = 50

# radians
kShootingAngleFudgeAmount = 0.005
# Elevator constants, replace values

kElevator1CANID = 55
kElevator1Name = "Elevator1Motor"
kElevator1PGain = 0.9
kElevator1IGain = 0
kElevator1DGain = 0
kElevator1Inverted = True

kElevatorMaxAccel = 390
kElevatorMaxVel = 300

kElevator2CANID = 56
kElevator2Name = "Elevator2Motor"
kElevator2PGain = 0.12
kElevator2IGain = 0
kElevator2DGain = 0
kElevator2Inverted = False

kElevatorTolerance = 0.05  # meters


kElevatorPositionKey = "elevator/position"
kElevatorStateKey = "elevator/state"
kElevatorAtPositionKey = "elevator/atPosition"

kElevatorPoseArrayKey = "elevator/pose"

kRobotToElevatorTransform = Transform3d(
    9.75 * kMetersPerInch,
    -6.75 * kMetersPerInch,
    13.875 * kMetersPerInch,
    Rotation3d(0, 0, math.pi),
)

kMotorPulleyGearRatio = 60 / 18 * 4 / 1

kPulleyGearPitchDiameter = 1.504 * kMetersPerInch
"""meters"""

kBottomPositionBeltPosition = -0.75 * kMetersPerInch
kAmpPositionBeltPosition = 19.125 * kMetersPerInch
kTopPositionBeltPosition = 28 * kMetersPerInch
"""meters"""

kBeltPullDownSpeed = 3
"""inches per second"""

kPullDownBandLimit = 0.1
"""Revolutions"""

kElevatorPositionKey = "ElevatorPosition"

kElevatorManualChange = 0.01
""".01 meters / (1/50) seconds = 0.5 m/s"""

kRobotToShooterTransform = Transform3d(
    Pose3d(),
    Pose3d(
        -5.631 * kMetersPerInch, 0, 13.191 * kMetersPerInch, Rotation3d(0, 0, math.pi)
    ),
)

kShooterWheelDiameter = 4 * kMetersPerInch
kSimNoteArrayKey = "SimNoteArray"
kLatestNoteTrajectoryKey = "LatestNoteTrajectory"
kNoteTrajectoryTimeInterval = 0.15
kShooterWheelRadius = kShooterWheelDiameter / 2
kShooterMovingIterations = 5

kAlignAnglePGain = 1.1
kAlignAngleIGain = 0
kAlignAngleDGain = 0

kAutoAimPGain = 0.6
kAutoAimIGain = 0
kAutoAimDGain = 0

kRotationAlignDeadband = Rotation2d.fromDegrees(1)

kPathfindingConstraints = PathConstraints(
    kMaxWheelLinearVelocity,
    kMaxWheelLinearAcceleration,
    kMaxRotationAngularVelocity,
    kMaxRotationAngularAcceleration,
)

kAmpWaypointBlue = Pose2d(1.828, 8.208 - kRobotLength / 2, math.pi / 2)
kSpeakerWaypointBlue = Pose2d(0.9067 + kRobotLength / 2, 5.551, 0)
kSourceWaypointBlue = Pose2d(
    15.62 - kRobotLength / 2 * math.cos(math.pi / 3),
    1.081 + kRobotLength / 2 * math.sin(math.pi / 3),
    -math.pi / 3,
)
kWaypointsBlue = [kAmpWaypointBlue, kSpeakerWaypointBlue, kSourceWaypointBlue]

kAmpWaypointRed = Pose2d(14.69, 8.208 - kRobotLength / 2, math.pi / 2)
kSpeakerWaypointRed = Pose2d(15.61 - kRobotLength / 2, 5.551, math.pi)
kSourceWaypointRed = Pose2d(
    0.8951 + kRobotLength / 2 * math.cos(math.pi / 3),
    1.081 + kRobotLength / 2 * math.sin(math.pi / 3),
    -2 * math.pi / 3,
)
kWaypointsRed = [kAmpWaypointRed, kSpeakerWaypointRed, kSourceWaypointRed]

kRobotToIntakePickupTransform = Transform2d(
    Pose2d(),
    Pose2d(
        0.446088,
        0.222375 - 0.342900 / 2,
        Rotation2d.fromDegrees(math.tan((0.222375 - 0.342900 / 2) / 0.446088)),
    ),
)
kAutoNotePickupAngleTolerance = Rotation2d.fromDegrees(10)
kNoteCameraDebounceTime = 1  # seconds
kMaxAutoNotePickupSpeed = 0.5  # 0 to 1

kAutoNotePickupPGain = 0.5
kAutoNotePickupIGain = 0
kAutoNotePickupDGain = 0

kSpeakerDistanceKey = "SpeakerDistance"

# NEEDS TEST DATA
kShooterAngleAdjustmentMappingFunction = (
    lambda x: 0.989 - 3.64 * x + 3.79 * x * x + 1.01 * x * x * x - 2.31 * x * x * x * x
)
# kShooterAngleAdjustmentMappingFunction = lambda x: 0

kIntakeRealZero = -0.25

# Climber stuff
kClimberCANID = 58
kClimberPIDSlot = 0
kClimberPGain = 0.9
kClimberIGain = 0
kClimberDGain = 0
kClimberInverted = False
kClimberName = "Climber Motor"

kClimberMotorPercent = 0

kClimberGearRatio = (3 / 1) * (3 / 1) * (44 / 28)

kClimberMapFunction = lambda x: x * (0.7 + 0.5 * kMetersPerInch) / 0.55
kClimberMapInverseFunction = lambda x: x * 0.55 / 0.7

kClimberStateKey = "climber/state"

kClimbingTopHeight = 26.5 * kMetersPerInch
kClimbingRetractedHeight = 0 * kMetersPerInch
kClimberHeightOffset = 0 * kMetersPerInch
kClimberHeightExtendOffsetElevator = 1.625 * kMetersPerInch

kClimberFloorTargetElevatorExtra = 0.875 * kMetersPerInch

kProfiledControllerPGain = 0.6
kProfiledControllerIGain = 0.0
kProfiledControllerDGain = 0.0

kProfiledMaxVelocityRetract = 3 * kMetersPerFoot  # m / s
kProfiledMaxVelocityExtend = 30 * kMetersPerFoot  # m / s
kProfiledMaxAccleration = 50 * kMetersPerFoot  # m / s / s

# shout out Ivan for this fr
kClimberHeightKey = "climber/ClimberHeight"
kClimberTargetKey = "climber/ClimberTarget"
kClimberPositionKey = "climber/ClimberPosition"
kClimberCurrentLimit = (
    CurrentLimitsConfigs()
    .with_stator_current_limit(35)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(35)
    .with_supply_current_limit_enable(True)
)  # copied from drive current limits cuz temp
kRobotToClimberTransform = Transform3d(
    -12.375 * kMetersPerInch,
    -0.138 * kMetersPerInch,
    8.5 * kMetersPerInch,
    Rotation3d(0, 0, math.pi),
)

kClimberWinchRadius = 0.787402 / 2 * kMetersPerInch  # get actual value ltr
