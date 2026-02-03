package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    double posRad = 0;
    double velPerSec = 0;
    double appliedVolts = 0;
    double currentAmps = 0;
  }


  default void updateInputs(KickerIOInputs inputs) {}

  public default void setKickerVoltage(double volts) {}

  public default void setKickerSpeed(double position) {}

  public default void stopKicker() {}

}
