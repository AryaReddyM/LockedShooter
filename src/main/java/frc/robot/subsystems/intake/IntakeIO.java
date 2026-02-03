package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    
    double rollerPosRad = 0;
    double rollerVelPerSec = 0;
    double rollerAppliedVolts = 0;
    double rollerCurrentAmps = 0;

    double extensionPosRad = 0;
    double extensionVelPerSec = 0;
    double extensionAppliedVolts = 0;
    double extensionCurrentAmps = 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}
}