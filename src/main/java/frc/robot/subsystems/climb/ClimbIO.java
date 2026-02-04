package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public boolean climbConnected = false;
    public double climbPositionRad = 0.0;
    public double climbVelocityRadPerSec = 0.0;
    public double climbAppliedVolts = 0.0;
    public double climbCurrentAmps = 0.0;
  }

  public static class ClimbIOOutputs {}

  default void updateInputs(ClimbIOInputs inputs) {}

  public default void setClimbVoltage(double volts) {}

  public default void setClimbPosition(double position) {}

  public default void stopClimb() {}

}
