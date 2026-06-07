package frc.robot.subsystems.hopper;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose3d;
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

public class HopperConstants {

    public static final int kHopperCanID = 15;


    // Hopper PID
    public static final double kHopperP = 0.2;
    public static final double kHopperI = 0;
    public static final double kHopperD = 0;
    public static final double kHopperMaxAccel = 200;
    public static final double kHopperCruiseVel = 1000;
    public static final double kHopperDeviationErr = 0;


    // factors
    public static final double kHopperPositionConversionFactor = 1.0/9.0;
    public static final double kHopperVelocityConversionFactor = 0.0001852;


    // Configuration
    public static final boolean kHopperinverted = false;
    public static final int kHopperCurrentLimit = 40;

    public static final double kRollerRadiusMeters = 0.0254; // change

    // setpoints
    public static final double kHopperShootSpeed = -15;
    public static final double kHopperOuttakeSpeed = 15;
    
    public static final Transform3d hopperOrigin = new Transform3d(new Translation3d(
        Units.inchesToMeters(2.5), Units.inchesToMeters(0.4), Units.inchesToMeters(0)
    ), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));

    public static MotorIO createIO() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.robot == Constants.RobotType.PRIMARY) {
                    return new MotorIOTalonFX(kHopperCanID, Constants.kCANivoreBus, talonConfig());
                }
                return MotorIOSpark.flex(kHopperCanID, flexConfig());
            case SIM:
                return MotorIOSim.flywheel(DCMotor.getNeo550(1), 0.025, 1.0, kHopperP, 0.0, 0.0789);
            default: // REPLAY
                return new MotorIO() {};
        }
    }

    private static SparkFlexConfig flexConfig() {
        SparkFlexConfig c = new SparkFlexConfig();
        c.inverted(kHopperinverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kHopperCurrentLimit)
                .voltageCompensation(12.0);
        c.encoder
                .positionConversionFactor(kHopperPositionConversionFactor)
                .velocityConversionFactor(kHopperVelocityConversionFactor)
                .quadratureAverageDepth(10)
                .quadratureMeasurementPeriod(2);
        c.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .pid(kHopperP, kHopperI, kHopperD)
                .maxMotion
                .maxAcceleration(kHopperMaxAccel)
                .cruiseVelocity(kHopperCruiseVel)
                .allowedProfileError(kHopperDeviationErr);
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
                kHopperinverted,
                true,
                kHopperCurrentLimit,
                2.0 * Math.PI / kHopperPositionConversionFactor,
                kHopperP,
                0.0,
                0.0,
                0.0,
                kHopperCruiseVel,
                kHopperMaxAccel);
    }
}
