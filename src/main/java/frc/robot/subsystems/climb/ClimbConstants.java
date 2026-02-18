package frc.robot.subsystems.climb;

public class ClimbConstants {

    public static final int kClimbCanID = 17;

    // Climb PID
    public static final double kClimbP = 0.1;
    public static final double kClimbI = 0;
    public static final double kClimbD = 0;
    public static final double kClimbMaxAccel = 1000;
    public static final double kClimbCruiseVel = 2000;
    public static final double kClimbDeviationErr = 0.05;

    public static final double kClimbSimP = 0.002;
    public static final double kClimbSimD = 0;

    public static final double kClimberBaseHeight = 0.4;

    // factors
    public static final double kClimbPositionConversionFactor = 1.0 / 16.0;
    public static final double kClimbVelocityConversionFactor = (1.0 / 16.0) / 60.0;

    // Configuration
    public static final boolean kClimbinverted = false;
    public static final int kClimbCurrentLimit = 30;

    // setpoints
    public static final double kClimbStowPos = 0;
    public static final double kClimbUpPos = 0.25;
    public static final double kClimbDownPos = 0.05;


    public static final double kLowerCurrentLimit = 6;
    public static final double kLowerMotorOutput = -0.1;
    public static final double kZeroCurrentThreshold = 4;
    
}
