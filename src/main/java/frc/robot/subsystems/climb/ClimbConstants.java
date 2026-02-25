package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

    public static final double kClimbSimP = 0.002;
    public static final double kClimbSimD = 0;

    public static final double kClimberBaseHeight = 0.4;

    // factors
    public static final double kClimbPositionConversionFactor = 1.0 / 16.0;
    public static final double kClimbVelocityConversionFactor = (1.0 / 16.0) / 60.0;

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
}
