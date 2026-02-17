package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public Rotation2d turretRotation2d = new Rotation2d();
    public double turretVelRadPerSec = 0.0;
    public double turretPos = 0.0;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public double desiredPos = 0.0;

    public Pose3d robotTurretPos = new Pose3d();
  }

  default void updateInputs(TurretIOInputs inputs) {}


  public default void setTurretVoltage(double volts) {}

  public default void setTurretPosition(double position, double ff) {}

  public default double getTurretPosition() {return 0.0;}
  public default double getTurretVelocity() {return 0.0;}
  public default Rotation2d getRobotToTurretRotation() {return new Rotation2d();}

  public default void stopTurret() {}
}
