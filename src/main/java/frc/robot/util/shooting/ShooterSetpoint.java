package frc.robot.util.shooting;

public class ShooterSetpoint {
  private final double shooterRPS;
  private final double hoodRadians;
  private final double hoodFF;
  private final double turretRadiansFromCenter;
  private final double turretFF;

  public ShooterSetpoint(
      double shooterRPS,
      double hoodRadians,
      double hoodFF,
      double turretRadiansFromCenter,
      double turretFF) {
    this.shooterRPS = shooterRPS;
    this.hoodRadians = hoodRadians;
    this.hoodFF = hoodFF;
    this.turretRadiansFromCenter = turretRadiansFromCenter;
    this.turretFF = turretFF;
  }

  public static ShooterSetpoint idle() {
    return new ShooterSetpoint(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public double getShooterRPS() {
    return shooterRPS;
  }

  public double getHoodRadians() {
    return hoodRadians;
  }

  public double getHoodFF() {
    return hoodFF;
  }

  public double getTurretRadiansFromCenter() {
    return turretRadiansFromCenter;
  }

  public double getTurretFF() {
    return turretFF;
  }
}
