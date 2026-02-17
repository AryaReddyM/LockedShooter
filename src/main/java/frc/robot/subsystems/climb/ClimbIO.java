package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {

    double posRad = 0;
    double velPerSec = 0;
    double appliedVolts = 0;
    double currentAmps = 0;
    
    double desiredPos = 0;
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  public default void setClimbVoltage(double volts) {}

  public default void setClimbPosition(double position) {}

  public default void stopClimb() {}

}
