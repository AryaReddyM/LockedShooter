package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class HoodConstants {

    public static final int kHoodCanID = 54;

    // Hood PID
    public static final double kHoodP = 0.8;
    public static final double kHoodI = 0;
    public static final double kHoodD = 0;
    public static final double kHoodS = 0.195;
    public static final double kHoodV = 0;
    public static final double kHoodA = 0;
    public static final double kHoodG = 0.401;
    public static final double kHoodMaxAccel = 100000;
    public static final double kHoodCruiseVel = 3000;
    public static final double kHoodDeviationErr = 0;

    public static final double kHoodSimP = 0.7;
    public static final double kHoodSimD = 0.2;

    // factors
    public static final double kHoodPositionConversionFactor = 2.0 * Math.PI / 3.0 / (367.0 / 32.0);
    public static final double kHoodVelocityConversionFactor = (2.0 * Math.PI / 3.0 / (367.0 / 32.0)) / 60.0;

    // Configuration
    public static final boolean kHoodinverted = false;
    public static final int kHoodCurrentLimit = 40;

    public static final double kHoodMaxSetpointUnderTrench = Units.degreesToRadians(25.0);
    public static final double kHoodMinLimit = Units.degreesToRadians(25.0);
    public static final double kHoodMaxLimit = Units.degreesToRadians(25.0) + kHoodMinLimit;

    // setpoints
    public static final Transform3d turretToHood = new Transform3d(new Translation3d(
        Units.inchesToMeters(4.145), Units.inchesToMeters(0.954), Units.inchesToMeters(2.260)
    ), new Rotation3d());
}
