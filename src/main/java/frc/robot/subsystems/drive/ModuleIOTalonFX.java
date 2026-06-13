package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder canCoder;
  private final Rotation2d zeroRotation;

  private final StatusSignal<Angle> drivePositionRot;
  private final StatusSignal<AngularVelocity> driveVelocityRps;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrentAmps;

  private final StatusSignal<Angle> turnPositionRot;
  private final StatusSignal<AngularVelocity> turnVelocityRps;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrentAmps;
  private final StatusSignal<Angle> canCoderAbsoluteRot;

  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut turnVoltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0);
  private final PositionVoltage turnPositionRequest = new PositionVoltage(0.0);

  public ModuleIOTalonFX(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };

    CANBus canBus = new CANBus(Constants.kCANivoreBus);
    driveTalon = new TalonFX(driveCanId(module), canBus);
    turnTalon = new TalonFX(turnCanId(module), canBus);
    canCoder = new CANcoder(canCoderId(module), canBus);

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = zeroRotation.getRotations();
    canCoder.getConfigurator().apply(canCoderConfig);

    driveTalon.getConfigurator().apply(driveConfig());
    turnTalon.getConfigurator().apply(turnConfig());

    drivePositionRot = driveTalon.getPosition();
    driveVelocityRps = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrentAmps = driveTalon.getStatorCurrent();

    turnPositionRot = turnTalon.getPosition();
    turnVelocityRps = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrentAmps = turnTalon.getStatorCurrent();
    canCoderAbsoluteRot = canCoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        odometryFrequency, drivePositionRot, turnPositionRot, canCoderAbsoluteRot);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocityRps,
        driveAppliedVolts,
        driveCurrentAmps,
        turnVelocityRps,
        turnAppliedVolts,
        turnCurrentAmps);

    BaseStatusSignal.refreshAll(canCoderAbsoluteRot);
    turnTalon.setPosition(canCoderAbsoluteRot.getValueAsDouble());
    driveTalon.setPosition(0.0);

    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    canCoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePositionRot,
        driveVelocityRps,
        driveAppliedVolts,
        driveCurrentAmps,
        turnPositionRot,
        turnVelocityRps,
        turnAppliedVolts,
        turnCurrentAmps,
        canCoderAbsoluteRot);

    inputs.driveConnected = true;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrentAmps.getValueAsDouble();

    inputs.turnConnected = true;
    inputs.turnPosition = Rotation2d.fromRotations(turnPositionRot.getValueAsDouble());
    inputs.canPosition = Rotation2d.fromRotations(canCoderAbsoluteRot.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrentAmps.getValueAsDouble();

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(driveVoltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(turnVoltageRequest.withOutput(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveTalon.setControl(
        driveVelocityRequest
            .withFeedForward(ffVolts));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpointRot =
        MathUtil.inputModulus(rotation.getRotations(), turnPIDMinInput / (2.0 * Math.PI),
            turnPIDMaxInput / (2.0 * Math.PI));
    turnTalon.setControl(turnPositionRequest.withPosition(setpointRot));
  }

  private static TalonFXConfiguration driveConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = driveMotorReduction;
    config.Slot0.kP = driveKp;
    return config;
  }

  private static TalonFXConfiguration turnConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        turnInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = turnMotorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = turnMotorReduction;
    config.ClosedLoopGeneral.ContinuousWrap = true;
    config.Slot0.kP = turnKp;
    config.Slot0.kV = turnKv;
    return config;
  }

  private static int driveCanId(int module) {
    return switch (module) {
      case 0 -> frontLeftDriveCanId;
      case 1 -> frontRightDriveCanId;
      case 2 -> backLeftDriveCanId;
      case 3 -> backRightDriveCanId;
      default -> 0;
    };
  }

  private static int turnCanId(int module) {
    return switch (module) {
      case 0 -> frontLeftTurnCanId;
      case 1 -> frontRightTurnCanId;
      case 2 -> backLeftTurnCanId;
      case 3 -> backRightTurnCanId;
      default -> 0;
    };
  }

  private static int canCoderId(int module) {
    return switch (module) {
      case 0 -> frontLeftCanCoderId;
      case 1 -> frontRightCanCoderId;
      case 2 -> backLeftCanCoderId;
      case 3 -> backRightCanCoderId;
      default -> 0;
    };
  }
}
