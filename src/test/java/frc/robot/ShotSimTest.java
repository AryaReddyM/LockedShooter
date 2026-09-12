package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.shooting.ShooterSetpoint;
import frc.robot.util.shooting.ShotCalculator;
import frc.robot.util.sim.FuelSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Fires shots the way the robot does and checks the fuel physics actually scores them. This is the
 * end to end check on the shot solver: if the maths and the simulated ball disagree, the hub score
 * does not move.
 */
class ShotSimTest {
  private static final Translation3d kHub = new Translation3d(
          VisionConstants.kBlueHubPose.getX(),
          VisionConstants.kBlueHubPose.getY(),
          VisionConstants.Hub.height);
  private static final double kLaunchHeight = 0.6;
  private static final double kClearance =
      VisionConstants.Hub.width / 2.0 + FuelSim.FUEL_RADIUS;

  /** Enough steps for a lofted shot to land. */
  private static final int kFlightLoops = 150;

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
  }

  private static FuelSim freshSim(Pose2d pose, ChassisSpeeds fieldSpeeds) {
    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
    FuelSim sim = new FuelSim();
    sim.registerRobot(
        DriveConstants.trackWidth,
        DriveConstants.wheelBase,
        DriveConstants.kBumperHeight,
        () -> pose,
        () -> fieldSpeeds);
    sim.enableAirResistance();
    sim.start();
    return sim;
  }

  private static ShooterSetpoint solveHub(Pose2d pose, ChassisSpeeds fieldSpeeds) {
    Translation3d launch = new Translation3d(pose.getX(), pose.getY(), kLaunchHeight);
    return ShotCalculator.solve(launch, pose, fieldSpeeds, kHub, kClearance, true);
  }

  /** Fires one shot and returns how many the blue hub took. */
  private static int fireAndCount(Pose2d pose, ChassisSpeeds fieldSpeeds, ShooterSetpoint shot) {
    FuelSim sim = freshSim(pose, fieldSpeeds);
    sim.launchFuel(
        MetersPerSecond.of(shot.getBallExitSpeed()),
        Radians.of(shot.getHoodRadians()),
        Radians.of(shot.getTurretRadiansFromCenter()),
        Meters.of(kLaunchHeight));
    for (int i = 0; i < kFlightLoops; i++) {
      sim.updateSim();
      if (FuelSim.Hub.BLUE_HUB.getScore() > 0) {
        break;
      }
    }
    return FuelSim.Hub.BLUE_HUB.getScore();
  }

  private static void assertScoresFrom(Pose2d pose) {
    assertScoresFrom(pose, new ChassisSpeeds());
  }

  private static void assertScoresFrom(Pose2d pose, ChassisSpeeds fieldSpeeds) {
    ShooterSetpoint shot = solveHub(pose, fieldSpeeds);
    assertTrue(
        shot.isAchievable(),
        "solver should find a shot from " + pose + " moving " + fieldSpeeds);
    assertEquals(
        1,
        fireAndCount(pose, fieldSpeeds, shot),
        "fuel should have scored from " + pose + " moving " + fieldSpeeds);
  }

  @Test
  void theAimPointMatchesWhereTheHubActuallyScores() {
    // The hub poses in VisionConstants sit at the base of the funnel, not the rim the
    // fuel has to cross, and aiming at the wrong one puts every shot into the near wall.
    assertEquals(
        FuelSim.Hub.ENTRY_HEIGHT,
        kHub.getZ(),
        0.01,
        "shots must be aimed at the height the hub scores at");
  }

  @Test
  void scoresFromStraightOnAtVariousDistances() {
    for (double x : new double[] {0.6, 1.0, 1.5, 2.0}) {
      assertScoresFrom(new Pose2d(x, kHub.getY(), Rotation2d.kZero));
    }
  }

  @Test
  void refusesShotsFromInsideTheMinimumRange() {
    // With the hood topping out at 50 degrees the fuel cannot be brought down steeply
    // enough to clear the rim from closer than about 2.6 m.
    for (double d : new double[] {1.5, 2.0, 2.4}) {
      ShooterSetpoint shot =
          solveHub(new Pose2d(kHub.getX() - d, kHub.getY(), Rotation2d.kZero), new ChassisSpeeds());
      assertFalse(shot.isAchievable(), "should not claim a shot is on from " + d + " m");
    }
  }

  @Test
  void everyShotItClaimsItCanMakeActuallyGoesIn() {
    int taken = 0;
    for (double d = 2.6; d <= 4.5; d += 0.1) {
      Pose2d pose = new Pose2d(kHub.getX() - d, kHub.getY(), Rotation2d.kZero);
      ShooterSetpoint shot = solveHub(pose, new ChassisSpeeds());
      if (!shot.isAchievable()) {
        continue;
      }
      taken++;
      assertEquals(1, fireAndCount(pose, new ChassisSpeeds(), shot), "missed from " + d + " m");
    }
    assertTrue(taken >= 15, "expected a usable range of shooting distances, got " + taken);
  }

  @Test
  void scoresFromOffToTheSide() {
    assertScoresFrom(new Pose2d(1.6, kHub.getY() - 2.2, Rotation2d.kZero));
    assertScoresFrom(new Pose2d(1.6, kHub.getY() + 2.2, Rotation2d.kZero));
    assertScoresFrom(new Pose2d(2.6, kHub.getY() - 3.0, Rotation2d.kZero));
    assertScoresFrom(new Pose2d(2.6, kHub.getY() + 3.0, Rotation2d.kZero));
  }

  @Test
  void scoresWhileTheChassisPointsAnyWhichWay() {
    for (double headingDeg : new double[] {0, 90, 180, -90, 143}) {
      assertScoresFrom(new Pose2d(1.5, kHub.getY() - 1.0, Rotation2d.fromDegrees(headingDeg)));
    }
  }

  @Test
  void scoresWhileDrivingSideways() {
    Pose2d pose = new Pose2d(1.5, kHub.getY() - 1.0, Rotation2d.fromDegrees(30));
    assertScoresFrom(pose, new ChassisSpeeds(0.0, 2.0, 0.0));
    assertScoresFrom(pose, new ChassisSpeeds(1.5, -1.5, 0.0));
    assertScoresFrom(pose, new ChassisSpeeds(-2.0, 0.0, 0.0));
  }

  @Test
  void chassisMotionActuallyChangesTheAim() {
    Pose2d pose = new Pose2d(1.5, kHub.getY(), Rotation2d.kZero);
    ShooterSetpoint still = solveHub(pose, new ChassisSpeeds());
    ShooterSetpoint moving = solveHub(pose, new ChassisSpeeds(0.0, 2.5, 0.0));

    assertTrue(
        Math.abs(still.getTurretRadiansFromCenter() - moving.getTurretRadiansFromCenter()) > 0.05,
        "driving sideways should make the turret lead the target");
  }

  @Test
  void solvedAnglesStayWithinTheMechanismLimits() {
    for (double x = 0.8; x <= 2.4; x += 0.2) {
      ShooterSetpoint shot = solveHub(new Pose2d(x, kHub.getY(), Rotation2d.kZero), new ChassisSpeeds());
      if (!shot.isAchievable()) {
        continue;
      }
      assertTrue(
          shot.getHoodRadians() >= HoodConstants.kHoodMinLimit - 1e-9
              && shot.getHoodRadians() <= HoodConstants.kHoodMaxLimit + 1e-9,
          "hood angle out of range at x=" + x + ": " + shot.getHoodRadians());
      assertTrue(
          shot.getTurretRadiansFromCenter() >= TurretConstants.kBackwardSoftLimit
              && shot.getTurretRadiansFromCenter() <= TurretConstants.kForwardSoftLimit,
          "turret angle out of range at x=" + x);
    }
  }

  @Test
  void turretAlwaysEndsUpInsideItsTravel() {
    for (double headingDeg = -180; headingDeg < 180; headingDeg += 15) {
      double wrapped =
          ShotCalculator.wrapIntoTurretRange(Math.toRadians(headingDeg));
      assertTrue(
          wrapped >= TurretConstants.kBackwardSoftLimit
              && wrapped <= TurretConstants.kForwardSoftLimit,
          "heading " + headingDeg + " wrapped to " + wrapped + ", outside the turret's travel");
      assertEquals(
          Math.cos(Math.toRadians(headingDeg)), Math.cos(wrapped), 1e-9, "wrap changed the heading");
      assertEquals(
          Math.sin(Math.toRadians(headingDeg)), Math.sin(wrapped), 1e-9, "wrap changed the heading");
    }
  }
}
