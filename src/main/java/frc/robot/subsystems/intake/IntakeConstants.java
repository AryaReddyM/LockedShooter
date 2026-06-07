package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.base.MotorConfigs;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOSim;
import frc.robot.subsystems.base.MotorIOSpark;
import frc.robot.subsystems.base.MotorIOTalonFX;

public class IntakeConstants {

    
    public static final int kRollersCanID = 48;
    public static final int kExtensionCanID = 46;
    public static final int kExtensionFollowerCanID = 47;

    // Roller PID
    public static final double kRollerP = 0.022;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;

    public static final double kRollerSimP = 0;
    public static final double kRollerSimD = 0;

    // Extension PID

    public static final double kExtensionP = 0.09;
    public static final double kExtensionI = 0;
    public static final double kExtensionD = 0;
    public static final double kExtensionMaxAccel = 600;
    public static final double kExtensionCruiseVel = 130;
    public static final double kExtensionDeviationErr = 2;
    public static final double kExtensionS = 0.17;
    public static final double kExtensionV = 0.00131;
    public static final double kExtensionA = 0;
    public static final double kExtensionCos = 0.24;

    public static final double kExtensionSimP = 0.3;
    public static final double kExtensionSimD = 0;


    // factors
    public static final double kRollerPositionConversionFactor = 1.0;
    public static final double kRollerVelocityConversionFactor = 1.0/60.0;

    public static int kRollerCurrentLimit = 70;

    public static final double kExtensionPositionConversionFactor = 360.0/23.0;
    public static final double kExtensionVelocityConversionFactor = (360.0/23.0) / 60.0;

    public static int kExtensionCurrentLimit = 60;
    // setpoints

    public static final double kExtensionStowSetpoint = -93;
    public static final double kExtensionIntakeSetpoint = 0;
    public static final double kExtensionOuttakeSetpoint = 0;
    public static final double kExtensionClimbTowSetpoint = -50;
    public static final double kExtensionShakeSetpoint = -30;

    public static final double kExtensionMax = 0;
    public static final double kExtensionMin = -96;

    public static final double kRollerOuttakeSpeed = 40;
    public static final double kRollerIntakeSpeed = -40;

    public static final Transform3d intakeOrigin = new Transform3d(new Translation3d(
        Units.inchesToMeters(5),Units.inchesToMeters(0), Units.inchesToMeters(6.7)
    ), new Rotation3d(0, Units.degreesToRadians(0), 0));

    private static SparkMax extensionFollower;

    public static MotorIO createExtensionIO() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.robot == Constants.RobotType.PRIMARY) {
                    return new MotorIOTalonFX(kExtensionCanID, Constants.kCANivoreBus, extensionTalonConfig());
                }
                configureFollower();
                return MotorIOSpark.max(kExtensionCanID, extensionConfig());
            case SIM:
                return MotorIOSim.arm(
                        DCMotor.getNeo550(1), 1.0, 0.5, 0.3, -1000.0, 1000.0, false, 0.0, kExtensionSimP);
            default: // REPLAY
                return new MotorIO() {};
        }
    }

    public static MotorIO createRollersIO() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.robot == Constants.RobotType.PRIMARY) {
                    return new MotorIOTalonFX(kRollersCanID, Constants.kCANivoreBus, rollerTalonConfig());
                }
                return MotorIOSpark.flex(kRollersCanID, rollerConfig());
            case SIM:
                return MotorIOSim.flywheel(DCMotor.getNeoVortex(1), 0.025, 1.0, kRollerSimP, 0.0, 0.0789);
            default: // REPLAY
                return new MotorIO() {};
        }
    }

    private static SparkMaxConfig extensionConfig() {
        SparkMaxConfig c = new SparkMaxConfig();
        c.inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kExtensionCurrentLimit)
                .voltageCompensation(12.0);
        c.encoder
                .positionConversionFactor(kExtensionPositionConversionFactor)
                .velocityConversionFactor(kExtensionVelocityConversionFactor)
                .quadratureAverageDepth(10)
                .quadratureMeasurementPeriod(2);
        c.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kExtensionP, kExtensionI, kExtensionD)
                .maxMotion
                .maxAcceleration(kExtensionMaxAccel)
                .cruiseVelocity(kExtensionCruiseVel)
                .allowedProfileError(kExtensionDeviationErr);
        c.closedLoop.feedForward.kS(kExtensionS).kV(kExtensionV).kCos(kExtensionCos);
        c.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        return c;
    }

    private static SparkFlexConfig rollerConfig() {
        SparkFlexConfig c = new SparkFlexConfig();
        c.inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kRollerCurrentLimit)
                .voltageCompensation(12.0);
        c.encoder
                .positionConversionFactor(kRollerPositionConversionFactor)
                .velocityConversionFactor(kRollerVelocityConversionFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        c.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kRollerP, kRollerI, kRollerD)
                .iMaxAccum(0.01);
        c.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        return c;
    }

    private static void configureFollower() {
        extensionFollower = new SparkMax(kExtensionFollowerCanID, MotorType.kBrushless);
        SparkMaxConfig c = new SparkMaxConfig();
        c.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kExtensionCurrentLimit)
                .voltageCompensation(12.0)
                .follow(kExtensionCanID, true);
        c.encoder
                .positionConversionFactor(kExtensionPositionConversionFactor)
                .velocityConversionFactor(kExtensionVelocityConversionFactor);
        c.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        extensionFollower.configure(c, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extensionFollower.clearFaults();
    }

    private static com.ctre.phoenix6.configs.TalonFXConfiguration extensionTalonConfig() {
        return MotorConfigs.talon(
                false,
                true,
                kExtensionCurrentLimit,
                2.0 * Math.PI / kExtensionPositionConversionFactor,
                kExtensionP,
                kExtensionS,
                kExtensionV,
                0.0,
                kExtensionCruiseVel,
                kExtensionMaxAccel);
    }

    private static com.ctre.phoenix6.configs.TalonFXConfiguration rollerTalonConfig() {
        return MotorConfigs.talon(
                false,
                true,
                kRollerCurrentLimit,
                2.0 * Math.PI / kRollerPositionConversionFactor,
                kRollerP,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0);
    }
}
