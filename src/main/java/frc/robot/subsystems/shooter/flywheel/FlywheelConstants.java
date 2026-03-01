package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class FlywheelConstants {

    public static final double kPassMaxApexHeight = Units.inchesToMeters(160);

    public static final int kFlywheelCanID = 55;
    public static final int kFlywheelFollowerCanID = 56;



    // Flywheel PID
    public static final double kFlywheelP = 0.4;
    public static final double kFlywheelI = 0;
    public static final double kFlywheelD = 0;
    public static final double kFlywheelMaxAccel = 100000000;
    public static final double kFlywheelCruiseVel = 100000000;
    public static final double kFlywheelDeviationErr = 0.01;


    // factors
    public static final double kFlywheelPositionConversionFactor = 1.0;
    public static final double kFlywheelVelocityConversionFactor = 1.0 / 60.0;

    public static final Distance kFlywheelRadius = Inches.of(2);


    // Configuration
    public static final boolean kFlywheelinverted = false;
    public static final int kFlywheelCurrentLimit = 40;

    public static final double kFlywheelSpeedTolerance = 0.4;

    // setpoints
    public static final double kSlowSpeed = 60.0;
}
