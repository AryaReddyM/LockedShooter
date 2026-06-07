package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

public class HoodConstants {

    public static final int kHoodCanID = 54;

    // Hood PID
    public static final double kHoodP = 0.8;
    public static final double kHoodI = 0;
    public static final double kHoodD = 0;
    public static final double kHoodS = 0.195;
    public static final double kHoodV = 0;
    public static final double kHoodA = 0;
    public static final double kHoodG = 0.401;
    public static final double kHoodMaxAccel = 100000;
    public static final double kHoodCruiseVel = 3000;
    public static final double kHoodDeviationErr = 0;

    public static final double kHoodSimP = 0.7;
    public static final double kHoodSimD = 0.2;

    // factors
    public static final double kHoodPositionConversionFactor = 2.0 * Math.PI / 3.0 / (367.0 / 32.0);
    public static final double kHoodVelocityConversionFactor = (2.0 * Math.PI / 3.0 / (367.0 / 32.0)) / 60.0;

    // Configuration
    public static final boolean kHoodinverted = false;
    public static final int kHoodCurrentLimit = 40;

    public static final double kHoodMaxSetpointUnderTrench = Units.degreesToRadians(25.0);
    public static final double kHoodMinLimit = Units.degreesToRadians(25.0);
    public static final double kHoodMaxLimit = Units.degreesToRadians(25.0) + kHoodMinLimit;

    // setpoints
    public static final Transform3d turretToHood = new Transform3d(new Translation3d(
        Units.inchesToMeters(4.145), Units.inchesToMeters(0.954), Units.inchesToMeters(2.260)
    ), new Rotation3d());

    public static MotorIO createIO() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.robot == Constants.RobotType.PRIMARY) {
                    return new MotorIOTalonFX(kHoodCanID, Constants.kCANivoreBus, talonConfig());
                }
                return MotorIOSpark.max(kHoodCanID, sparkConfig());
            case SIM:
                return MotorIOSim.arm(
                        DCMotor.getNeo550(1), 1.0, 0.5, 0.3, kHoodMinLimit, kHoodMaxLimit, false, kHoodMinLimit,
                        kHoodSimP);
            default: // REPLAY
                return new MotorIO() {};
        }
    }

    private static SparkMaxConfig sparkConfig() {
        SparkMaxConfig c = new SparkMaxConfig();
        c.inverted(kHoodinverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kHoodCurrentLimit)
                .voltageCompensation(12.0);
        c.encoder
                .positionConversionFactor(kHoodPositionConversionFactor)
                .velocityConversionFactor(kHoodVelocityConversionFactor)
                .quadratureAverageDepth(10)
                .quadratureMeasurementPeriod(2);
        c.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kHoodP, kHoodI, kHoodD)
                .maxMotion
                .maxAcceleration(kHoodMaxAccel)
                .cruiseVelocity(kHoodCruiseVel)
                .allowedProfileError(kHoodDeviationErr);
        c.closedLoop.feedForward.kS(kHoodS).kV(kHoodV).kA(kHoodA).kG(kHoodG);
        c.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        c.softLimit
                .forwardSoftLimit(kHoodMaxLimit)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(kHoodMinLimit)
                .reverseSoftLimitEnabled(true);
        return c;
    }

    private static com.ctre.phoenix6.configs.TalonFXConfiguration talonConfig() {
        return MotorConfigs.talon(
                kHoodinverted,
                true,
                kHoodCurrentLimit,
                2.0 * Math.PI / kHoodPositionConversionFactor,
                kHoodP,
                kHoodS,
                kHoodV,
                kHoodG,
                kHoodCruiseVel,
                kHoodMaxAccel);
    }
}
