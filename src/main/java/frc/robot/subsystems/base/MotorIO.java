package frc.robot.subsystems.base;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(MotorIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setPosition(double positionRad) {}

  default void setPosition(double positionRad, double feedforwardVolts) {
    setPosition(positionRad);
  }

  default void setMotionMagicPosition(double positionRad) {}

  default void setMotionMagicPosition(double positionRad, double feedforwardVolts) {
    setMotionMagicPosition(positionRad);
  }

  default void setVelocity(double velocityRadPerSec) {}

  default void setVelocity(double velocityRadPerSec, double feedforwardVolts) {
    setVelocity(velocityRadPerSec);
  }

  default void setMotionMagicVelocity(double velocityRadPerSec) {}

  default void setMotionMagicVelocity(double velocityRadPerSec, double feedforwardVolts) {
    setMotionMagicVelocity(velocityRadPerSec);
  }

  default void setMotionMagicPosition(double positionRad, int slot) {
    setMotionMagicPosition(positionRad);
  }

  // ----- Misc / specialty ------------------------------------------------------------------------

  default void setDutyCycle(double fraction) {}

  default void setCurrentLimit(double amps) {}

  default void stop() {}

  default void setEncoderPosition(double positionRad) {}

  default void setBrakeMode(boolean enabled) {}
}
