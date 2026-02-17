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

  @AutoLog
    public static class SetpointLog {
        public double shooterRPS = 0.0;
        public double shooterStage1RPS = 14.4;
        public double turretRadiansFromCenter = 0.0;
        public double turretFF = 0.0;
        public double hoodRadians = 0.0;
        public double hoodFF = 0.0;
        public double height = 0.0;
        public boolean isValid = true;        
    }

  default void updateInputs(TurretIOInputs inputs) {}


  public default void setTurretVoltage(double volts) {}

  public default void setTurretPosition(double position, double ff) {}

  public default double getTurretPosition() {return 0.0;}
  public default double getTurretVelocity() {return 0.0;}
  public default Rotation2d getRobotToTurretRotation() {return new Rotation2d();}

  public default void stopTurret() {}
}
