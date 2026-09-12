package frc.robot.subsystems.climb;

import com.revrobotics.spark.ClosedLoopSlot;
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

public class ClimbConstants {

    public static final int kClimbCanID = 13;

    // Climb PID
    public static final double kClimbP = 20;
    public static final double kClimbI = 0;
    public static final double kClimbD = 0;

    public static final double kClimbMaxAccel = 4000;
    public static final double kClimbCruiseVel = 7000;
    public static final double kClimbDeviationErr = 0.05;

    public static final double kClimbActionP = 7;

    public static final double kClimbActionMaxAccel = 600;
    public static final double kClimbActionCruiseVel = 600;

    // Sim. The climb reports output rotations, so gains are volts per rotation.
    public static final double kClimbSimP = 6.0;
    public static final double kClimbSimD = 0.4;
    public static final double kClimbSimArmLengthMeters = 0.5;
    public static final double kClimbSimMassKg = 3.0;
    public static final double kClimbRotationsPerRadian = 1.0 / (2.0 * Math.PI);

    public static final double kClimberBaseHeight = 0.4;

    // factors
    public static final double kClimbPositionConversionFactor = 1.0 / 16.0;
    public static final double kClimbVelocityConversionFactor = kClimbPositionConversionFactor / 60.0;

    /** Motor rotations per output rotation, implied by the position conversion factor. */
    public static final double kClimbReduction = 1.0 / kClimbPositionConversionFactor;

    // Configuration
    public static final boolean kClimbinverted = false;
    public static final int kClimbCurrentLimit = 50;

    // setpoints
    public static final double kClimbStowPos = 0;
    public static final double kClimbUpPos = 4.1;
    public static final double kClimbDownPos = 1.5;

    public static final double kLowerCurrentLimit = 30;
    public static final double kLowerMotorOutput = -0.08;
    public static final double kZeroCurrentThreshold = 30;

    public static final int kBeamBreakerIdOne = 1;
    public static final int kBeamBreakerIdTwo = 2;


    public static final Transform3d climbOrigin = new Transform3d(new Translation3d(
        Units.inchesToMeters(0.938), Units.inchesToMeters(12.733), Units.inchesToMeters(1.621)
    ), new Rotation3d());

    public static MotorIO createIO() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.robot == Constants.RobotType.PRIMARY) {
                    return new MotorIOTalonFX(kClimbCanID, Constants.kCANivoreBus, talonConfig());
                }
                return MotorIOSpark.flex(kClimbCanID, flexConfig());
            case SIM:
                // A winch, so no gravity term and generous travel limits either side of the
                // stow and deploy positions.
                return MotorIOSim.arm(
                        DCMotor.getNeoVortex(1),
                        kClimbReduction,
                        kClimbSimArmLengthMeters,
                        kClimbSimMassKg,
                        -1.0,
                        kClimbUpPos + 1.0,
                        false,
                        kClimbStowPos,
                        kClimbRotationsPerRadian,
                        kClimbSimP,
                        kClimbSimD);
            default:
                return new MotorIO() {};
        }
    }

    private static SparkFlexConfig flexConfig() {
        SparkFlexConfig c = new SparkFlexConfig();
        c.inverted(kClimbinverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kClimbCurrentLimit)
                .voltageCompensation(12.0);
        c.encoder
                .positionConversionFactor(kClimbPositionConversionFactor)
                .velocityConversionFactor(kClimbVelocityConversionFactor);
        c.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kClimbP, kClimbI, kClimbD, ClosedLoopSlot.kSlot0)
                .pid(kClimbActionP, kClimbI, kClimbD, ClosedLoopSlot.kSlot1);
        c.closedLoop.maxMotion
                .maxAcceleration(kClimbMaxAccel, ClosedLoopSlot.kSlot0)
                .cruiseVelocity(kClimbCruiseVel, ClosedLoopSlot.kSlot0)
                .maxAcceleration(kClimbActionMaxAccel, ClosedLoopSlot.kSlot1)
                .cruiseVelocity(kClimbActionCruiseVel, ClosedLoopSlot.kSlot1)
                .allowedProfileError(kClimbDeviationErr, ClosedLoopSlot.kSlot0)
                .allowedProfileError(kClimbDeviationErr, ClosedLoopSlot.kSlot1);
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
                kClimbinverted,
                true,
                kClimbCurrentLimit,
                2.0 * Math.PI / kClimbPositionConversionFactor,
                kClimbP,
                0.0,
                0.0,
                0.0,
                kClimbCruiseVel,
                kClimbMaxAccel);
    }
}
