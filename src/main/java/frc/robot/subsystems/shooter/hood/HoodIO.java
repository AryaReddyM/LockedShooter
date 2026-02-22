package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    double posRad = 0;
    double velPerSec = 0;
    double appliedVolts = 0;
    double currentAmps = 0;

    double desiredPos = 0;
  }


  default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodVoltage(double volts) {}

  public default void setHoodPosition(double position, double ff) {}
  public default double getHoodPosition() {return 0.0;}

  public default void stopHood() {}
}
