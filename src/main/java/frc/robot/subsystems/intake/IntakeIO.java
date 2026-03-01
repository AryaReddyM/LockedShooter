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

    double extensionFollowerPosRad = 0;
    double extensionFollowerVelPerSec = 0;
    double extensionFollowerAppliedVolts = 0;
    double extensionFollowerCurrentAmps = 0;

    double desiredExtensionPos = 0;
  }


  default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVoltage(double volts) {}

  public default void setRollerSpeed(double speed) {}

  public default void setExtensionPosition(double position) {}

  public default void stopRollers() {}

  public default void stopExtension() {}


}
