package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {

    public static final int kHoodCanID = 0;

    // Hood PID
    public static final double kHoodP = 0;
    public static final double kHoodI = 0;
    public static final double kHoodD = 0;
    public static final double kHoodMaxAccel = 0;
    public static final double kHoodCruiseVel = 0;
    public static final double kHoodDeviationErr = 0;

    public static final double kHoodSimP = 0.7;
    public static final double kHoodSimD = 0.2;

    // factors
    public static final double kHoodPositionConversionFactor = 0;
    public static final double kHoodVelocityConversionFactor = 0;

    // Configuration
    public static final boolean kHoodinverted = false;
    public static final int kHoodCurrentLimit = 10;

    // setpoints
    public static final double kHoodStowPos = 0;
//TODO
    public static final double kHoodPositionTolerance = 0.1;
    public static final double kHoodMinPositionRadians = 0.0;
    public static final double kHoodZeroedAngleDegrees = 0;
    public static final double kHoodEpsilon = Units.degreesToRadians(1.0);
    public static final double kHoodShootingEpsilon = Units.degreesToRadians(5.0);

    public static final double kFenderShotRadians = Units.degreesToRadians(0);

}
