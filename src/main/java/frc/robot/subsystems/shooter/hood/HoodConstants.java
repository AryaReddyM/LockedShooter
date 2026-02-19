package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static final Transform3d turretToHood = new Transform3d(new Translation3d(
        Units.inchesToMeters(4.145), Units.inchesToMeters(0.954), Units.inchesToMeters(2.260)
    ), new Rotation3d());
}
