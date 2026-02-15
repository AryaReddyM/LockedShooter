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

    // factors
    public static final double kClimbPositionConversionFactor = 1.0 / 16.0;
    public static final double kClimbVelocityConversionFactor = (1.0 / 16.0) / 60.0;

    // Configuration
    public static final boolean kClimbinverted = false;
    public static final int kClimbCurrentLimit = 30;

    // setpoints
    public static final double kClimbStowPos = 0;
    public static final double kClimbUpPos = 2.0;
    public static final double kClimbDownPos = 1.0;
    
}
