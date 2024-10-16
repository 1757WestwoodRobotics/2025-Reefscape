from commands2.command import Command
from wpilib import Preferences

from subsystems.shootersubsystem import ShooterSubsystem


import constants


class SubwooferShot(Command):
    def __init__(self, shooterSubsystem: ShooterSubsystem):
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.shooter = shooterSubsystem

        self.addRequirements(shooterSubsystem)

    def execute(self):
        self.shooter.setShooterAngle(constants.kShooterSubwooferAngle)

        spinAmount = Preferences.getDouble("Spin Amount", 100)
        self.shooter.setLeftShootingMotorSpeed(
            constants.kShooterSubwooferSpeed - spinAmount
        )
        self.shooter.setRightShootingMotorSpeed(
            constants.kShooterSubwooferSpeed + spinAmount
        )

    def isFinished(self) -> bool:
        return self.shooter.readyToShoot()


class PodiumShot(Command):
    def __init__(self, shooterSubsystem: ShooterSubsystem):
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.shooter = shooterSubsystem

        self.addRequirements(shooterSubsystem)

    def execute(self):
        self.shooter.setShooterAngle(constants.kPodiumShooterAngle)

        spinAmount = Preferences.getDouble("Spin Amount", 100)
        self.shooter.setLeftShootingMotorSpeed(
            constants.kPodiumShooterSpeed - spinAmount
        )
        self.shooter.setRightShootingMotorSpeed(
            constants.kPodiumShooterSpeed + spinAmount
        )


class PassShot(Command):
    def __init__(self, shooterSubsystem: ShooterSubsystem):
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.shooter = shooterSubsystem

        self.addRequirements(shooterSubsystem)

    def execute(self):
        self.shooter.safePivot()

        self.shooter.setLeftShootingMotorSpeed(constants.kShooterPassSpeed)
        self.shooter.setRightShootingMotorSpeed(constants.kShooterPassSpeed)


class SafetyPosition(Command):
    def __init__(self, shooterSubsystem: ShooterSubsystem):
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.shooter = shooterSubsystem

        self.addRequirements(shooterSubsystem)

    def execute(self):
        self.shooter.neutralShooter()

    def isFinished(self) -> bool:
        return self.shooter.angleOnTarget()
