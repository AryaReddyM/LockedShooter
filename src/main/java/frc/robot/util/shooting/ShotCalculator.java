package frc.robot.util.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;

/**
 * Turns "where is the robot and where is the target" into a {@link ShooterSetpoint}.
 *
 * <p>The trajectory is solved in the field frame, then the robot's own velocity is subtracted from
 * the required launch vector. That is what makes shooting on the move work: the fuel keeps the
 * chassis velocity it had when it left the shooter, so the shooter only has to supply the
 * difference between the trajectory it wants and the motion it already has.
 *
 * <p>Because of that subtraction the trajectory's elevation and the hood's angle are not the same
 * number once the robot is moving, so the search runs over field frame elevations and keeps the
 * ones whose <i>hood</i> angle lands inside the hood's travel.
 */
public final class ShotCalculator {
  /** Elevation angles considered, in the field frame. */
  private static final double kMinElevation = Math.toRadians(10.0);

  private static final double kMaxElevation = Math.toRadians(80.0);
  private static final int kElevationSamples = 71;

  /**
   * Slack left at each end of the hood's travel while screening. Solving for drag nudges the speed
   * up a few percent, which moves the hood angle slightly, and this keeps that drift from landing
   * just outside a limit.
   */
  private static final double kHoodMargin = Math.toRadians(1.0);

  private ShotCalculator() {}

  /** The launch vector broken into what the shooter has to produce. */
  private record ShooterAim(double speed, double hoodRad, double turretFieldRad) {}

  /**
   * @param launchPoint where the fuel leaves the robot, in field coordinates
   * @param robotPose current robot pose, used for the heading the turret is measured against
   * @param fieldRelativeSpeeds robot velocity in field coordinates
   * @param target the point the fuel should pass through, in field coordinates
   * @param clearanceRadiusMeters how far in front of the target the fuel already has to be at
   *     target height, so it drops in over the structure instead of hitting the near wall of it.
   *     Zero for an open target such as a pass.
   * @param withDrag whether the fuel simulation is modelling air resistance
   */
  public static ShooterSetpoint solve(
      Translation3d launchPoint,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      Translation3d target,
      double clearanceRadiusMeters,
      boolean withDrag) {
    double dx = target.getX() - launchPoint.getX();
    double dy = target.getY() - launchPoint.getY();
    double distance = Math.hypot(dx, dy);
    double heightGain = target.getZ() - launchPoint.getZ();

    if (distance < 1e-3) {
      return ShooterSetpoint.idle();
    }

    double maxExitSpeed =
        FlywheelConstants.surfaceSpeedToExitVelocity(FlywheelConstants.kMaxSurfaceSpeed);

    double elevation =
        chooseElevation(
            distance,
            heightGain,
            clearanceRadiusMeters,
            dx / distance,
            dy / distance,
            fieldRelativeSpeeds,
            maxExitSpeed);
    if (Double.isNaN(elevation)) {
      return unreachable(robotPose, dx, dy);
    }

    BallisticSolver.Solution solution =
        BallisticSolver.solveAtAngle(distance, heightGain, elevation, maxExitSpeed, withDrag);
    if (!solution.achievable()
        || !clears(
            distance,
            heightGain,
            clearanceRadiusMeters,
            solution.speedMetersPerSec(),
            elevation,
            withDrag)
        || !BallisticSolver.isDescendingAtTarget(
            distance, solution.speedMetersPerSec(), elevation, withDrag)) {
      return unreachable(robotPose, dx, dy);
    }

    ShooterAim aim =
        toShooterFrame(
            solution.speedMetersPerSec(),
            elevation,
            dx / distance,
            dy / distance,
            fieldRelativeSpeeds);

    double turretRobot =
        wrapIntoTurretRange(
            MathUtil.angleModulus(aim.turretFieldRad() - robotPose.getRotation().getRadians()));

    boolean withinHood =
        aim.hoodRad() >= HoodConstants.kHoodMinLimit - 1e-6
            && aim.hoodRad() <= HoodConstants.kHoodMaxLimit + 1e-6;
    boolean withinTurret =
        turretRobot >= TurretConstants.kBackwardSoftLimit
            && turretRobot <= TurretConstants.kForwardSoftLimit;
    boolean withinFlywheel = aim.speed() <= maxExitSpeed;

    double surfaceSpeed =
        MathUtil.clamp(
            FlywheelConstants.exitVelocityToSurfaceSpeed(aim.speed()),
            0.0,
            FlywheelConstants.kMaxSurfaceSpeed);

    return new ShooterSetpoint(
        surfaceSpeed,
        MathUtil.clamp(aim.hoodRad(), HoodConstants.kHoodMinLimit, HoodConstants.kHoodMaxLimit),
        HoodConstants.kHoodG,
        MathUtil.clamp(
            turretRobot, TurretConstants.kBackwardSoftLimit, TurretConstants.kForwardSoftLimit),
        0.0,
        withinHood && withinTurret && withinFlywheel);
  }

  /**
   * Splits a field frame launch vector into the part the shooter has to supply, given that the
   * chassis is already carrying the fuel along with it.
   */
  private static ShooterAim toShooterFrame(
      double fieldSpeed,
      double elevationRad,
      double unitX,
      double unitY,
      ChassisSpeeds fieldRelativeSpeeds) {
    double horizontal = fieldSpeed * Math.cos(elevationRad);
    double vertical = fieldSpeed * Math.sin(elevationRad);

    double shooterVx = horizontal * unitX - fieldRelativeSpeeds.vxMetersPerSecond;
    double shooterVy = horizontal * unitY - fieldRelativeSpeeds.vyMetersPerSecond;
    double shooterHorizontal = Math.hypot(shooterVx, shooterVy);

    return new ShooterAim(
        Math.hypot(shooterHorizontal, vertical),
        Math.atan2(vertical, shooterHorizontal),
        Math.atan2(shooterVy, shooterVx));
  }

  /**
   * Picks the field frame elevation whose shot the flywheel can make most slowly, out of those that
   * drop in cleanly and leave the hood inside its travel.
   *
   * <p>Screening is drag free because that is closed form and cheap; drag raises the speed every
   * elevation needs by roughly the same proportion, so it barely moves which one wins, and the
   * chosen elevation then gets a full solve.
   *
   * @return the best elevation in radians, or NaN if nothing works from here
   */
  private static double chooseElevation(
      double distance,
      double heightGain,
      double clearanceRadiusMeters,
      double unitX,
      double unitY,
      ChassisSpeeds fieldRelativeSpeeds,
      double maxExitSpeed) {
    double best = Double.NaN;
    double bestShooterSpeed = Double.POSITIVE_INFINITY;

    for (int i = 0; i < kElevationSamples; i++) {
      double elevation =
          kMinElevation + (kMaxElevation - kMinElevation) * i / (kElevationSamples - 1.0);

      // Arriving on the descending branch requires d*tan(elevation) > 2*heightGain.
      if (distance * Math.tan(elevation) <= 2.0 * heightGain) {
        continue;
      }
      double fieldSpeed = BallisticSolver.solveSpeedNoDrag(distance, heightGain, elevation);
      if (Double.isNaN(fieldSpeed)) {
        continue;
      }
      double edge = distance - clearanceRadiusMeters;
      if (clearanceRadiusMeters > 0.0
          && edge > 0.0
          && BallisticSolver.heightAtDistanceNoDrag(edge, fieldSpeed, elevation) < heightGain) {
        continue;
      }

      ShooterAim aim =
          toShooterFrame(fieldSpeed, elevation, unitX, unitY, fieldRelativeSpeeds);
      if (aim.hoodRad() < HoodConstants.kHoodMinLimit + kHoodMargin
          || aim.hoodRad() > HoodConstants.kHoodMaxLimit - kHoodMargin) {
        continue;
      }
      if (aim.speed() > maxExitSpeed || aim.speed() >= bestShooterSpeed) {
        continue;
      }
      bestShooterSpeed = aim.speed();
      best = elevation;
    }
    return best;
  }

  /**
   * Whether the fuel is already at target height by the time it reaches the near edge of the
   * structure around the target, rather than still climbing up into the side of it.
   */
  private static boolean clears(
      double distance,
      double heightGain,
      double clearanceRadiusMeters,
      double speed,
      double angleRad,
      boolean withDrag) {
    double edge = distance - clearanceRadiusMeters;
    if (clearanceRadiusMeters <= 0.0 || edge <= 0.0) {
      return true;
    }
    return BallisticSolver.heightAtDistance(edge, speed, angleRad, withDrag) >= heightGain;
  }

  /** Keeps the turret pointed at the target while flagging the shot as not takeable. */
  private static ShooterSetpoint unreachable(Pose2d robotPose, double dx, double dy) {
    double turretRobot =
        wrapIntoTurretRange(
            MathUtil.angleModulus(Math.atan2(dy, dx) - robotPose.getRotation().getRadians()));
    return new ShooterSetpoint(
        0.0,
        HoodConstants.kHoodMinLimit,
        HoodConstants.kHoodG,
        MathUtil.clamp(
            turretRobot, TurretConstants.kBackwardSoftLimit, TurretConstants.kForwardSoftLimit),
        0.0,
        false);
  }

  /**
   * The turret's travel spans a full turn but is not centred on zero, so an angle wrapped to
   * [-pi, pi) can still fall outside it. Shifting by a full turn reaches the same heading from the
   * other side.
   */
  public static double wrapIntoTurretRange(double angleRad) {
    if (angleRad > TurretConstants.kForwardSoftLimit
        && angleRad - 2.0 * Math.PI >= TurretConstants.kBackwardSoftLimit) {
      return angleRad - 2.0 * Math.PI;
    }
    if (angleRad < TurretConstants.kBackwardSoftLimit
        && angleRad + 2.0 * Math.PI <= TurretConstants.kForwardSoftLimit) {
      return angleRad + 2.0 * Math.PI;
    }
    return angleRad;
  }
}
