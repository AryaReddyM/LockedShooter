package frc.robot.subsystems.base;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class MotorConfigs {
  private MotorConfigs() {}

  public static TalonFXConfiguration talon(
      boolean inverted,
      boolean brake,
      double currentLimit,
      double sensorToMechanismRatio,
      double kP,
      double kS,
      double kV,
      double kG,
      double motionMagicCruise,
      double motionMagicAccel) {
    TalonFXConfiguration c = new TalonFXConfiguration();
    c.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    c.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    c.CurrentLimits.StatorCurrentLimit = currentLimit;
    c.CurrentLimits.StatorCurrentLimitEnable = true;
    c.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
    c.Slot0.kP = kP;
    c.Slot0.kS = kS;
    c.Slot0.kV = kV;
    c.Slot0.kG = kG;
    c.MotionMagic.MotionMagicCruiseVelocity = motionMagicCruise;
    c.MotionMagic.MotionMagicAcceleration = motionMagicAccel;
    return c;
  }

  public static TalonFX follower(int followerCanId, String canBus, int leaderCanId, boolean opposeLeader) {
    TalonFX follower = new TalonFX(followerCanId, new CANBus(canBus));
    follower.setControl(
        new Follower(
            leaderCanId,
            opposeLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    return follower;
  }
}
