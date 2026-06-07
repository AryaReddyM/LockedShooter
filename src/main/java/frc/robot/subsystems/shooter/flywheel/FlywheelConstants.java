package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.subsystems.base.MotorConfigs;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOSim;
import frc.robot.subsystems.base.MotorIOSpark;
import frc.robot.subsystems.base.MotorIOTalonFX;

public class FlywheelConstants {

    public static final double kPassMaxApexHeight = Units.inchesToMeters(160);

    public static final int kFlywheelCanID = 55;
    public static final int kFlywheelFollowerCanID = 56;



    // Flywheel PID
    public static final double kFlywheelP = 0.5;
    public static final double kFlywheelI = 0;
    public static final double kFlywheelD = 0;
    public static final double kFlywheelS = 0.0;
    public static final double kFlywheelV = 0.38;
    public static final double kFlywheelA = 0.0;
    public static final double kFlywheelG = 0.0;
    public static final double kFlywheelMaxAccel = 100000000;
    public static final double kFlywheelCruiseVel = 100000000;
    public static final double kFlywheelDeviationErr = 0.01;


// factors
    public static final Distance kFlywheelRadius = Inches.of(2);

    public static final double kFlywheelPositionConversionFactor = kFlywheelRadius.in(Meters) * Math.PI * 2.0;
    public static final double kFlywheelVelocityConversionFactor = kFlywheelPositionConversionFactor / 60.0;


    // Configuration
    public static final boolean kFlywheelinverted = false;
    public static final int kFlywheelCurrentLimit = 60;

    public static final double kFlywheelSpeedTolerance = 2;

    // setpoints
    public static final double kSlowSpeed = 0.0;

    private static SparkFlex flywheelFollower;

    public static MotorIO createIO() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.robot == Constants.RobotType.PRIMARY) {
                    return new MotorIOTalonFX(kFlywheelCanID, Constants.kCANivoreBus, talonConfig());
                }
                configureFollower();
                return MotorIOSpark.flex(kFlywheelCanID, flexConfig());
            case SIM:
                return MotorIOSim.flywheel(DCMotor.getNeoVortex(1), 0.025, 1.0, kFlywheelP, 0.0, 0.0789);
            default: // REPLAY
                return new MotorIO() {};
        }
    }

    private static SparkFlexConfig flexConfig() {
        SparkFlexConfig c = new SparkFlexConfig();
        c.inverted(kFlywheelinverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(kFlywheelCurrentLimit)
                .voltageCompensation(12.0);
        c.encoder
                .positionConversionFactor(kFlywheelPositionConversionFactor)
                .velocityConversionFactor(kFlywheelVelocityConversionFactor)
                .quadratureAverageDepth(10)
                .quadratureMeasurementPeriod(2);
        c.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .outputRange(0, 1)
                .pid(kFlywheelP, kFlywheelI, kFlywheelD)
                .maxMotion
                .maxAcceleration(kFlywheelMaxAccel)
                .cruiseVelocity(kFlywheelCruiseVel)
                .allowedProfileError(kFlywheelDeviationErr);
        c.closedLoop.feedForward.kS(kFlywheelS).kV(kFlywheelV).kA(kFlywheelA).kG(kFlywheelG);
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
        flywheelFollower = new SparkFlex(kFlywheelFollowerCanID, MotorType.kBrushless);
        SparkFlexConfig c = new SparkFlexConfig();
        c.follow(kFlywheelCanID);
        flywheelFollower.configure(c, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelFollower.clearFaults();
    }

    private static com.ctre.phoenix6.configs.TalonFXConfiguration talonConfig() {
        return MotorConfigs.talon(
                kFlywheelinverted,
                false,
                kFlywheelCurrentLimit,
                2.0 * Math.PI / kFlywheelPositionConversionFactor,
                kFlywheelP,
                kFlywheelS,
                kFlywheelV,
                kFlywheelG,
                kFlywheelCruiseVel,
                kFlywheelMaxAccel);
    }
}
