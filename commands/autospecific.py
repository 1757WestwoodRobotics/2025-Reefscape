from commands2.sequentialcommandgroup import SequentialCommandGroup
from commands.intakesetting import FeedIntakeToShooter, FloorIntake, HoldIntakeAtHandoff
from commands.shooter.alignandaim import AlignAndAim
from commands.shooter.shooterfixedshots import SafetyPosition, SubwooferShot

AimAndFire = lambda shooter, drive, intake: SequentialCommandGroup(
    HoldIntakeAtHandoff(intake),
    AlignAndAim(shooter, drive, (lambda: 0), (lambda: 0)),
    FeedIntakeToShooter(intake),
)

IntakeAuto = lambda intake, shooter: SequentialCommandGroup(
    SafetyPosition(shooter), FloorIntake(intake)
)

SubwooferAuto = lambda intake, shooter: SequentialCommandGroup(
    SubwooferShot(shooter), FeedIntakeToShooter(intake)
)
