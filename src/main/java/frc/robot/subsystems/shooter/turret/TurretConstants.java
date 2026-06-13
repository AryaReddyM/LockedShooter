package frc.robot.subsystems.shooter.turret;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.base.MotorConfigs;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOSim;
import frc.robot.subsystems.base.MotorIOSpark;
import frc.robot.subsystems.base.MotorIOTalonFX;

public class TurretConstants {
  public static final int kTurretCanId = 24;

  public static final double kTurretP = 0.91;
  public static final double kTurretI = 0;
  public static final double kTurretD = 0;
  public static final double kTurretS = 0;
  public static final double kTurretV = 0;
  public static final double kTurretA = 0;
  public static final double kTurretG = 0;
  public static final double kTurretMaxAccel = 800;
  public static final double kTurretCruiseVel = 600;
  public static final double kTurretDeviationErr = 1;

  public static final Rotation2d kTurretAbsEncoderOffset = Rotation2d.fromRadians(0);

  public static final double kTurretSimP = 0.1;
  public static final double kTurretSimD = 0;

  // factors
  public static final double kTurretPositionConversionFactor = 2.0 * Math.PI / 4.0 / (200.0 / 20.0);
  public static final double kTurretVelocityConversionFactor =
      (2.0 * Math.PI / 4.0 / (200.0 / 20.0)) / 60;

  public static final double kReadyToleranceDegrees = 4.0;

  // Configuration
  public static final boolean kTurretinverted = true;
  public static final int kTurretCurrentLimit = 20;

  public static final double latencyCompensationMS = 20.0;

  public static final double kForwardSoftLimit = 2.7;
  public static final double kBackwardSoftLimit = -Math.PI - (Math.PI - kForwardSoftLimit);

  public static MotorIO createIO() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.robot == Constants.RobotType.PRIMARY) {
          return new MotorIOTalonFX(kTurretCanId, Constants.kCANivoreBus, talonConfig());
        }
        return MotorIOSpark.max(kTurretCanId, sparkConfig());
      case SIM:
        return MotorIOSim.arm(
            DCMotor.getNeo550(1),
            1.0,
            0.1,
            0.3,
            kBackwardSoftLimit,
            kForwardSoftLimit,
            false,
            0.0,
            kTurretSimP);
      default:
        return new MotorIO() {};
    }
  }

  private static SparkMaxConfig sparkConfig() {
    SparkMaxConfig c = new SparkMaxConfig();
    c.inverted(kTurretinverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kTurretCurrentLimit)
        .voltageCompensation(12.0);
    c.encoder
        .positionConversionFactor(kTurretPositionConversionFactor)
        .velocityConversionFactor(kTurretVelocityConversionFactor)
        .quadratureAverageDepth(10)
        .quadratureMeasurementPeriod(2);
    c.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingInputRange(kBackwardSoftLimit, kForwardSoftLimit)
        .positionWrappingEnabled(false)
        .pid(kTurretP, kTurretI, kTurretD)
        .maxMotion
        .maxAcceleration(kTurretMaxAccel)
        .cruiseVelocity(kTurretCruiseVel)
        .allowedProfileError(kTurretDeviationErr);
    c.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    c.softLimit
        .forwardSoftLimit(kForwardSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(kBackwardSoftLimit)
        .reverseSoftLimitEnabled(true);
    return c;
  }

  private static com.ctre.phoenix6.configs.TalonFXConfiguration talonConfig() {
    return MotorConfigs.talon(
        kTurretinverted,
        true,
        kTurretCurrentLimit,
        2.0 * Math.PI / kTurretPositionConversionFactor,
        kTurretP,
        kTurretS,
        kTurretV,
        kTurretG,
        kTurretCruiseVel,
        kTurretMaxAccel);
  }
}
