package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final Rotation2d kTurretToShotCorrection = new Rotation2d(Units.degreesToRadians(1.5));
    /// TODO
    public static final double kPassMaxApexHeight = Units.inchesToMeters(96.0);

    public static final double kStage2ShooterWheelDiameter = Units.inchesToMeters(3.0); // in
    public static final double kStage1ShooterWheelDiameter = Units.inchesToMeters(2.0); // in

    public static final double kBallLaunchVelMetersPerSecPerRotPerSec = 0.056;
    public static final double kBallLaunchLiftCoeff = 0.015; // Multiply by v^2 to get lift accel
    public static final double kShooterStage2RPSShortRange = 70.0; // rot/s
    public static final double kShooterStage2MaxShortRangeDistance = 1.5;
    public static final double kShooterStage2MinLongRangeDistance =1.5;
    public static final double kShooterStage2RPSLongRange = 130.0; // rot/s
    public static final double kShooterStage2RPSCap = 170.0; // rot/s

    public static final double kBallReleaseHeight = Units.inchesToMeters(22.183);
}
