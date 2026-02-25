package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    double posRad = 0;
    double velPerSec = 0;
    double appliedVolts = 0;
    double currentAmps = 0;
    
    boolean isReady = false;
  }


  default void updateInputs(FlywheelIOInputs inputs) {}


  public default void setFlywheelVoltage(double volts) {}

  public default void setFlywheelSpeed(double position, double ff) {}
   public default void setFlywheelSpeed(double position) {}


  public default void stopFlywheel() {}
  public default boolean isAtSpeed(double speed, double tolerance) {return false;};

}
