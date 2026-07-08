package frc.robot.subsystems.elevator;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.base.MotorConfigs;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOSim;
import frc.robot.subsystems.base.MotorIOSpark;
import frc.robot.subsystems.base.MotorIOTalonFX;

public class ElevatorConstants {
  public static final int kElevatorCanID = 30;

  public static final double kElevatorP = 4.0;
  public static final double kElevatorI = 0;
  public static final double kElevatorD = 0;
  public static final double kElevatorG = 0.3;
  public static final double kElevatorMaxAccel = 2.0;
  public static final double kElevatorCruiseVel = 1.5;
  public static final double kElevatorDeviationErr = 0;

  public static final double kElevatorSimP = 8.0;

  public static final double kDrumRadiusMeters = Units.inchesToMeters(1.0);
  public static final double kCarriageMassKg = 5.0;
  public static final double kGearRatio = 9.0;
  public static final double kPositionConversionFactor =
      (2.0 * Math.PI * kDrumRadiusMeters) / kGearRatio;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

  public static final boolean kInverted = false;
  public static final int kCurrentLimit = 40;

  public static final double kStowHeight = 0.0;
  public static final double kLowHeight = Units.inchesToMeters(12);
  public static final double kHighHeight = Units.inchesToMeters(36);
  public static final double kEpsilon = Units.inchesToMeters(0.5);

  public static MotorIO createIO() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.robot == Constants.RobotType.PRIMARY) {
          return new MotorIOTalonFX(kElevatorCanID, Constants.kCANivoreBus, talonConfig());
        }
        return MotorIOSpark.max(kElevatorCanID, sparkConfig());
      case SIM:
        return MotorIOSim.elevator(
            DCMotor.getNEO(1),
            kGearRatio,
            kCarriageMassKg,
            kDrumRadiusMeters,
            kStowHeight,
            kHighHeight + kEpsilon,
            kElevatorSimP);
      default:
        return new MotorIO() {};
    }
  }

  private static SparkMaxConfig sparkConfig() {
    SparkMaxConfig c = new SparkMaxConfig();
    c.inverted(kInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kCurrentLimit)
        .voltageCompensation(12.0);
    c.encoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor);
    c.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kElevatorP, kElevatorI, kElevatorD)
        .maxMotion
        .maxAcceleration(kElevatorMaxAccel)
        .cruiseVelocity(kElevatorCruiseVel)
        .allowedProfileError(kElevatorDeviationErr);
    c.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    return c;
  }

  private static com.ctre.phoenix6.configs.TalonFXConfiguration talonConfig() {
    return MotorConfigs.talon(
        kInverted,
        true,
        kCurrentLimit,
        2.0 * Math.PI / kPositionConversionFactor,
        kElevatorP,
        0.0,
        0.0,
        kElevatorG,
        kElevatorCruiseVel,
        kElevatorMaxAccel);
  }
}
