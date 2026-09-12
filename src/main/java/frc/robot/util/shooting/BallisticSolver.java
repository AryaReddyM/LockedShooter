package frc.robot.util.shooting;

import frc.robot.util.sim.FuelSim;

/**
 * Projectile solver for a piece of fuel, including quadratic air drag.
 *
 * <p>Everything here works in the field frame in a single vertical plane: {@code distance} is the
 * horizontal distance from the launch point to the target and {@code heightGain} is how far the
 * target sits above the launch point. The physics constants and the integration scheme are taken
 * straight from {@link FuelSim} so a solution computed here lands where the sim says it lands.
 */
public final class BallisticSolver {
  /** Integration step, matching one FuelSim subtick (20 ms / 5). */
  private static final double kStepSeconds = 0.004;

  private static final double kMaxFlightSeconds = 4.0;
  private static final int kMaxIterations = 24;
  private static final double kHeightToleranceMeters = 0.01;
  private static final double kDragPerMass = FuelSim.DRAG_FORCE_FACTOR / FuelSim.FUEL_MASS;

  private BallisticSolver() {}

  /**
   * A launch speed and elevation angle, plus whether the shot actually reaches the target on the
   * way down.
   */
  public record Solution(double speedMetersPerSec, double angleRad, boolean achievable) {
    public static Solution unreachable(double angleRad) {
      return new Solution(0.0, angleRad, false);
    }
  }

  /**
   * Closed-form launch speed for a drag-free shot at a fixed elevation angle.
   *
   * @return the required speed, or NaN when no positive solution exists at that angle
   */
  public static double solveSpeedNoDrag(double distance, double heightGain, double angleRad) {
    double cos = Math.cos(angleRad);
    double denominator = 2.0 * cos * cos * (distance * Math.tan(angleRad) - heightGain);
    if (denominator <= 0.0) {
      // The target is at or above the straight-line reach of this angle: no speed works.
      return Double.NaN;
    }
    return Math.sqrt(FuelSim.GRAVITY_MPS2 * distance * distance / denominator);
  }

  /**
   * Closed-form height, relative to the launch point, after travelling {@code distance}
   * horizontally with no drag. Used for the cheap screening pass.
   */
  public static double heightAtDistanceNoDrag(double distance, double speed, double angleRad) {
    double cos = Math.cos(angleRad);
    return distance * Math.tan(angleRad)
        - FuelSim.GRAVITY_MPS2 * distance * distance / (2.0 * speed * speed * cos * cos);
  }

  /**
   * Height of the ball, relative to the launch point, once it has travelled {@code distance}
   * horizontally. Returns {@link Double#NEGATIVE_INFINITY} if it falls short.
   *
   * <p>Integrated exactly the way {@link FuelSim} steps a fuel: advance the position, then apply
   * gravity and drag to the velocity.
   */
  public static double heightAtDistance(
      double distance, double speed, double angleRad, boolean withDrag) {
    double x = 0.0;
    double z = 0.0;
    double vx = speed * Math.cos(angleRad);
    double vz = speed * Math.sin(angleRad);

    for (double t = 0.0; t < kMaxFlightSeconds; t += kStepSeconds) {
      double prevX = x;
      double prevZ = z;
      x += vx * kStepSeconds;
      z += vz * kStepSeconds;

      if (x >= distance) {
        // Interpolate across the step that crossed the target plane.
        double span = x - prevX;
        double fraction = span > 1e-9 ? (distance - prevX) / span : 0.0;
        return prevZ + (z - prevZ) * fraction;
      }

      double ax = 0.0;
      double az = -FuelSim.GRAVITY_MPS2;
      if (withDrag) {
        double speedNow = Math.hypot(vx, vz);
        if (speedNow > 1e-6) {
          ax -= kDragPerMass * speedNow * vx;
          az -= kDragPerMass * speedNow * vz;
        }
      }
      vx += ax * kStepSeconds;
      vz += az * kStepSeconds;

      if (vx <= 0.0) {
        // Drag has killed the horizontal component; it will never get there.
        break;
      }
    }
    return Double.NEGATIVE_INFINITY;
  }

  /**
   * Finds the launch speed that puts the ball at {@code heightGain} exactly {@code distance} away,
   * at the given elevation angle.
   *
   * <p>Height at a fixed distance rises monotonically with launch speed, so this brackets the
   * drag-free answer from below and bisects. Drag only ever shortens a shot, so the drag-free speed
   * is always a valid lower bound.
   */
  public static Solution solveAtAngle(
      double distance, double heightGain, double angleRad, double maxSpeed, boolean withDrag) {
    if (distance <= 0.0) {
      return Solution.unreachable(angleRad);
    }

    double noDrag = solveSpeedNoDrag(distance, heightGain, angleRad);
    if (Double.isNaN(noDrag)) {
      return Solution.unreachable(angleRad);
    }
    if (!withDrag) {
      return new Solution(noDrag, angleRad, noDrag <= maxSpeed);
    }

    double low = noDrag;
    double high = Math.min(noDrag * 3.0, maxSpeed);
    if (heightAtDistance(distance, high, angleRad, true) < heightGain) {
      // Even at the highest speed available the ball lands short of the target.
      return Solution.unreachable(angleRad);
    }

    for (int i = 0; i < kMaxIterations; i++) {
      double mid = 0.5 * (low + high);
      double height = heightAtDistance(distance, mid, angleRad, true);
      if (Math.abs(height - heightGain) < kHeightToleranceMeters) {
        return new Solution(mid, angleRad, true);
      }
      if (height < heightGain) {
        low = mid;
      } else {
        high = mid;
      }
    }
    return new Solution(0.5 * (low + high), angleRad, true);
  }

  /**
   * True when a shot at this speed and angle is still on its way down as it reaches the target,
   * which is what the hub needs in order to count it. A ball that arrives while still climbing hits
   * the underside of the funnel lip instead of dropping through.
   */
  public static boolean isDescendingAtTarget(
      double distance, double speed, double angleRad, boolean withDrag) {
    double justBefore = heightAtDistance(distance - 0.15, speed, angleRad, withDrag);
    double atTarget = heightAtDistance(distance, speed, angleRad, withDrag);
    if (justBefore == Double.NEGATIVE_INFINITY || atTarget == Double.NEGATIVE_INFINITY) {
      return false;
    }
    return atTarget < justBefore;
  }
}
