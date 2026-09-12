package frc.robot.util.shooting;

import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;

/**
 * One fully specified shot.
 *
 * <p>Units follow what each mechanism actually reports:
 *
 * <ul>
 *   <li>{@code flywheelSurfaceSpeed} is the flywheel's <b>surface speed in m/s</b>, which is the
 *       native unit of the flywheel controller (its position conversion factor is the wheel
 *       circumference). The ball leaves at a fraction of that; see {@link
 *       FlywheelConstants#surfaceSpeedToExitVelocity(double)}.
 *   <li>{@code hoodRadians} is the launch angle measured <b>up from horizontal</b>.
 *   <li>{@code turretRadiansFromCenter} is <b>robot relative</b>, zero pointing straight ahead.
 * </ul>
 */
public class ShooterSetpoint {
  private final double flywheelSurfaceSpeed;
  private final double hoodRadians;
  private final double hoodFF;
  private final double turretRadiansFromCenter;
  private final double turretFF;
  private final boolean achievable;

  public ShooterSetpoint(
      double flywheelSurfaceSpeed,
      double hoodRadians,
      double hoodFF,
      double turretRadiansFromCenter,
      double turretFF) {
    this(flywheelSurfaceSpeed, hoodRadians, hoodFF, turretRadiansFromCenter, turretFF, true);
  }

  public ShooterSetpoint(
      double flywheelSurfaceSpeed,
      double hoodRadians,
      double hoodFF,
      double turretRadiansFromCenter,
      double turretFF,
      boolean achievable) {
    this.flywheelSurfaceSpeed = flywheelSurfaceSpeed;
    this.hoodRadians = hoodRadians;
    this.hoodFF = hoodFF;
    this.turretRadiansFromCenter = turretRadiansFromCenter;
    this.turretFF = turretFF;
    this.achievable = achievable;
  }

  public static ShooterSetpoint idle() {
    return new ShooterSetpoint(0.0, 0.0, 0.0, 0.0, 0.0, false);
  }

  /** Flywheel surface speed in m/s, the unit the flywheel controller is configured in. */
  public double getFlywheelSurfaceSpeed() {
    return flywheelSurfaceSpeed;
  }

  /** Speed the fuel leaves the shooter at, in m/s. */
  public double getBallExitSpeed() {
    return FlywheelConstants.surfaceSpeedToExitVelocity(flywheelSurfaceSpeed);
  }

  /** Launch angle above horizontal, in radians. */
  public double getHoodRadians() {
    return hoodRadians;
  }

  public double getHoodFF() {
    return hoodFF;
  }

  /** Robot relative turret angle, in radians, zero pointing straight ahead. */
  public double getTurretRadiansFromCenter() {
    return turretRadiansFromCenter;
  }

  public double getTurretFF() {
    return turretFF;
  }

  /** False when the target is out of range or outside the hood's travel. */
  public boolean isAchievable() {
    return achievable;
  }
}
