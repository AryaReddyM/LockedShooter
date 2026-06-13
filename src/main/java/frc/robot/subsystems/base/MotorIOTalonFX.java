package frc.robot.subsystems.base;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOTalonFX implements MotorIO {
  private final TalonFX talon;
  private final TalonFXConfiguration config;

  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> deviceTemp;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final MotionMagicVelocityVoltage motionMagicVelocityRequest = new MotionMagicVelocityVoltage(0.0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  public MotorIOTalonFX(int canId, String canBus, TalonFXConfiguration config) {
    this.config = config;
    talon = new TalonFX(canId, new CANBus(canBus));
    talon.getConfigurator().apply(config);

    positionRot = talon.getPosition();
    velocityRps = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    statorCurrent = talon.getStatorCurrent();
    deviceTemp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRps, appliedVolts, statorCurrent, deviceTemp);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRps, appliedVolts, statorCurrent, deviceTemp);
    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRps.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double positionRad) {
    setPosition(positionRad, 0.0);
  }

  @Override
  public void setPosition(double positionRad, double feedforwardVolts) {
    talon.setControl(
        positionRequest
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setMotionMagicPosition(double positionRad) {
    setMotionMagicPosition(positionRad, 0.0);
  }

  @Override
  public void setMotionMagicPosition(double positionRad, double feedforwardVolts) {
    talon.setControl(
        motionMagicRequest
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setMotionMagicPosition(double positionRad, int slot) {
    talon.setControl(
        motionMagicRequest
            .withPosition(Units.radiansToRotations(positionRad))
            .withSlot(slot));
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    setVelocity(velocityRadPerSec, 0.0);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double feedforwardVolts) {
    talon.setControl(
        velocityRequest
            .withVelocity(Units.radiansToRotations(velocityRadPerSec))
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setMotionMagicVelocity(double velocityRadPerSec) {
    setMotionMagicVelocity(velocityRadPerSec, 0.0);
  }

  @Override
  public void setMotionMagicVelocity(double velocityRadPerSec, double feedforwardVolts) {
    talon.setControl(
        motionMagicVelocityRequest
            .withVelocity(Units.radiansToRotations(velocityRadPerSec))
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setDutyCycle(double fraction) {
    talon.setControl(dutyCycleRequest.withOutput(fraction));
  }

  @Override
  public void setCurrentLimit(double amps) {
    config.CurrentLimits.StatorCurrentLimit = amps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    talon.getConfigurator().apply(config.CurrentLimits);
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void setEncoderPosition(double positionRad) {
    talon.setPosition(Units.radiansToRotations(positionRad));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talon.getConfigurator().apply(config.MotorOutput);
  }
}
