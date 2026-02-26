package frc.robot.subsystems.kicker;

public class KickerConstants {

    public static final int kKickerCanID = 42;


    // Kicker PID
    public static final double kKickerP = 0.02;
    public static final double kKickerI = 0;
    public static final double kKickerD = 0;
    public static final double kKickerMaxAccel = 10;
    public static final double kKickerCruiseVel = 200;
    public static final double kKickerDeviationErr = 0;


    // factors
    public static final double kKickerPositionConversionFactor = 1.0/3.0;
    public static final double kKickerVelocityConversionFactor = (1.0/3.0)/60.0;


    // Configuration
    public static final boolean kKickerinverted = false;
    public static final int kKickerCurrentLimit = 40;


    // setpoints
    public static final double kKickerShootSpeed = 6;
    public static final double kKickerOutakeSpeed = -6;
    
    
}
