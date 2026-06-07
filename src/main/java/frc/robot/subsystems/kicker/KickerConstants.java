package frc.robot.subsystems.kicker;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.base.MotorConfigs;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOSim;
import frc.robot.subsystems.base.MotorIOSpark;
import frc.robot.subsystems.base.MotorIOTalonFX;

public class KickerConstants {

    public static final int kKickerCanID = 42;


    // Kicker PID
    public static final double kKickerP = 0.025;
    public static final double kKickerI = 0;
    public static final double kKickerD = 0;
    public static final double kKickerMaxAccel = 100000;
    public static final double kKickerCruiseVel = 100000;
    public static final double kKickerDeviationErr = 0;


    // factors
    public static final double kKickerPositionConversionFactor = 1.0/3.0;
    public static final double kKickerVelocityConversionFactor = (1.0/3.0)/60.0;


    // Configuration
    public static final boolean kKickerinverted = false;
    public static final int kKickerCurrentLimit = 40;


    // setpoints
    public static final double kKickerShootSpeed = -40;
    public static final double kKickerOutakeSpeed = 30;

    public static MotorIO createIO() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.robot == Constants.RobotType.PRIMARY) {
                    return new MotorIOTalonFX(kKickerCanID, Constants.kCANivoreBus, talonConfig());
                }
                return MotorIOSpark.max(kKickerCanID, sparkConfig());
            case SIM:
                return MotorIOSim.flywheel(DCMotor.getNeo550(1), 0.025, 1.0, kKickerP, 0.0, 0.0789);
            default: // REPLAY
                return new MotorIO() {};
        }
    }

    private static SparkMaxConfig sparkConfig() {
        SparkMaxConfig c = new SparkMaxConfig();
        c.inverted(kKickerinverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kKickerCurrentLimit)
                .voltageCompensation(12.0);
        c.encoder
                .positionConversionFactor(kKickerPositionConversionFactor)
                .velocityConversionFactor(kKickerVelocityConversionFactor)
                .quadratureAverageDepth(10)
                .quadratureMeasurementPeriod(2);
        c.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .pid(kKickerP, kKickerI, kKickerD)
                .maxMotion
                .maxAcceleration(kKickerMaxAccel)
                .cruiseVelocity(kKickerCruiseVel)
                .allowedProfileError(kKickerDeviationErr);
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
                kKickerinverted,
                true,
                kKickerCurrentLimit,
                2.0 * Math.PI / kKickerPositionConversionFactor,
                kKickerP,
                0.0,
                0.0,
                0.0,
                kKickerCruiseVel,
                kKickerMaxAccel);
    }
}
