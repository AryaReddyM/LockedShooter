package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.ClosedLoopSlot;

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
  public default void setClimbPosition(double position, ClosedLoopSlot slot) {}

  public default void stopClimb() {}

  public default void setCurrentLimit(double limit) {}
  public default void setMotorOutput(double output) {}
  public default double getMotorCurrent() {return 5.0;};
  public default void zeroEncoder() {};

}
