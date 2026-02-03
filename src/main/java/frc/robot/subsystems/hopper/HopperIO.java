package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    double posRad = 0;
    double velPerSec = 0;
    double appliedVolts = 0;
    double currentAmps = 0;
  }

  default void updateInputs(HopperIOInputs inputs) {}

  public default void setHopperVoltage(double volts) {}

  public default void setHopperSpeed(double position) {}

  public default void stopHopper() {}

}
