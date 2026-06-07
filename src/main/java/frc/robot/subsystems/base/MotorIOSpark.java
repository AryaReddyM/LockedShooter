package frc.robot.subsystems.base;

import static frc.robot.util.hardware.SparkUtil.ifOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MotorIOSpark implements MotorIO {
  private final SparkBase spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkBaseConfig config;

  private MotorIOSpark(SparkBase spark, SparkBaseConfig config) {
    this.spark = spark;
    this.config = config;
    this.encoder = spark.getEncoder();
    this.controller = spark.getClosedLoopController();
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spark.clearFaults();
  }

  public static MotorIOSpark max(int canId, SparkMaxConfig config) {
    return new MotorIOSpark(new SparkMax(canId, MotorType.kBrushless), config);
  }

  public static MotorIOSpark flex(int canId, SparkFlexConfig config) {
    return new MotorIOSpark(new SparkFlex(canId, MotorType.kBrushless), config);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    ifOk(spark, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(spark, spark::getMotorTemperature, (value) -> inputs.tempCelsius = value);
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void setPosition(double positionRad) {
    controller.setSetpoint(positionRad, ControlType.kPosition);
  }

  @Override
  public void setPosition(double positionRad, double feedforwardVolts) {
    controller.setSetpoint(
        positionRad, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setMotionMagicPosition(double positionRad) {
    controller.setSetpoint(positionRad, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setMotionMagicPosition(double positionRad, double feedforwardVolts) {
    controller.setSetpoint(
        positionRad,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setMotionMagicPosition(double positionRad, int slot) {
    controller.setSetpoint(positionRad, ControlType.kMAXMotionPositionControl, slotOf(slot));
  }

  private static ClosedLoopSlot slotOf(int slot) {
    switch (slot) {
      case 1:
        return ClosedLoopSlot.kSlot1;
      case 2:
        return ClosedLoopSlot.kSlot2;
      case 3:
        return ClosedLoopSlot.kSlot3;
      default:
        return ClosedLoopSlot.kSlot0;
    }
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    controller.setSetpoint(velocityRadPerSec, ControlType.kVelocity);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double feedforwardVolts) {
    controller.setSetpoint(
        velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setMotionMagicVelocity(double velocityRadPerSec) {
    controller.setSetpoint(velocityRadPerSec, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setMotionMagicVelocity(double velocityRadPerSec, double feedforwardVolts) {
    controller.setSetpoint(
        velocityRadPerSec,
        ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setDutyCycle(double fraction) {
    spark.set(fraction);
  }

  @Override
  public void setCurrentLimit(double amps) {
    config.smartCurrentLimit((int) amps);
    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }

  @Override
  public void setEncoderPosition(double positionRad) {
    encoder.setPosition(positionRad);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
