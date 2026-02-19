package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class HopperConstants {

    public static final int kHopperCanID = 0;


    // Hopper PID
    public static final double kHopperP = 0;
    public static final double kHopperI = 0;
    public static final double kHopperD = 0;
    public static final double kHopperMaxAccel = 0;
    public static final double kHopperCruiseVel = 0;
    public static final double kHopperDeviationErr = 0;


    // factors
    public static final double kHopperPositionConversionFactor = 0;
    public static final double kHopperVelocityConversionFactor = 0;


    // Configuration
    public static final boolean kHopperinverted = false;
    public static final int kHopperCurrentLimit = 10;

    public static final double kRollerRadiusMeters = 0.0254; // change

    // setpoints
    public static final double kHopperShootSpeed = 0.4;
    public static final double kHopperOuttakeSpeed = -0.4;
    
    public static final Transform3d hopperOrigin = new Transform3d(new Translation3d(
        Units.inchesToMeters(0.36), Units.inchesToMeters(2.5), Units.inchesToMeters(9.5)
    ), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-90), Units.degreesToRadians(0)));
}
