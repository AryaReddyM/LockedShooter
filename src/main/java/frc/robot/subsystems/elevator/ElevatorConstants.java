package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOSim;
import frc.robot.subsystems.base.MotorIOSpark;
import frc.robot.subsystems.base.MotorIOTalonFX;

public class ElevatorConstants {
    // --- Hardware identity ---
    public static final int kElevatorCanID = 20;
    public static final String kCanBus = "rio";

    // --- Physical geometry ---
    public static final double kGearRatio = 9.0;                          // motor rotations : drum rotations
    public static final double kDrumRadiusMeters = Units.inchesToMeters(1.0);
    public static final double kCarriageMassKg = 4.5;
    public static final double kMinHeightMeters = 0.0;
    public static final double kMaxHeightMeters = Units.inchesToMeters(40.0);

    // --- Control gains ---
    public static final double kP = 24.0;          // real: volts per mechanism-position error
    public static final double kSimP = 60.0;       // sim: volts per meter error
    public static final double kG = 0.35;          // gravity feedforward, VOLTS (holds the carriage)

    // MotionMagic limits for the real controller (Phoenix mechanism-rotation units).
    public static final double kMMCruise = 0.16;   // ~1.0 m/s  (= m/s / 2pi)
    public static final double kMMAccel = 0.32;    // ~2.0 m/s^2

    public static final double kCurrentLimit = 60.0;
    public static final double kToleranceMeters = Units.inchesToMeters(0.5);

    // --- Named heights (meters) ---
    public static final double kStowMeters = 0.0;
    public static final double kLowMeters = Units.inchesToMeters(10.0);
    public static final double kMidMeters = Units.inchesToMeters(24.0);
    public static final double kHighMeters = Units.inchesToMeters(38.0);

    public static MotorIO createIO() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.robot == Constants.RobotType.PRIMARY) {
                    return new MotorIOTalonFX(kElevatorCanID, Constants.kCANivoreBus, talonConfig());
                }
                return MotorIOSpark.max(kElevatorCanID, sparkConfig());
            case SIM:
                return MotorIOSim.elevator(
                        DCMotor.getKrakenX60(1),
                        kGearRatio,
                        kCarriageMassKg,
                        kDrumRadiusMeters,
                        kMinHeightMeters,
                        kMaxHeightMeters,
                        kSimP);
            default: // REPLAY
                return new MotorIO() {};
        }
    }

    private static TalonFXConfiguration talonConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.StatorCurrentLimit = kCurrentLimit;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.Feedback.SensorToMechanismRatio = kGearRatio / kDrumRadiusMeters;
        cfg.Slot0.kP = kP;
        cfg.MotionMagic.MotionMagicCruiseVelocity = kMMCruise;
        cfg.MotionMagic.MotionMagicAcceleration = kMMAccel;
        return cfg;
    }

    private static SparkMaxConfig sparkConfig() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        double positionFactorMeters = 2.0 * Math.PI * kDrumRadiusMeters / kGearRatio;
        cfg.idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int) kCurrentLimit)
                .voltageCompensation(12.0);
        cfg.encoder
                .positionConversionFactor(positionFactorMeters)
                .velocityConversionFactor(positionFactorMeters / 60.0);
        cfg.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kP, 0.0, 0.0)
                .maxMotion
                .cruiseVelocity(kMMCruise)
                .maxAcceleration(kMMAccel);
        cfg.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        return cfg;
    }
}
