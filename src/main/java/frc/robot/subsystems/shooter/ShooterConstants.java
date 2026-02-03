package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final Rotation2d kTurretToShotCorrection = new Rotation2d(Units.degreesToRadians(1.5));
    /// TODO
    public static final double kPassMaxApexHeight = Units.inchesToMeters(160.0);

    public static final double kStage2ShooterWheelDiameter = Units.inchesToMeters(3.0); // in
    public static final double kStage1ShooterWheelDiameter = Units.inchesToMeters(2.0); // in

    public static final double kBallLaunchVelMetersPerSecPerRotPerSec = 0.141;
    public static final double kBallLaunchLiftCoeff = 0.013; // Multiply by v^2 to get lift accel
    public static final double kShooterStage2RPSShortRange = 120.0; // rot/s
    public static final double kShooterStage2MaxShortRangeDistance = 2.0;
    public static final double kShooterStage2MinLongRangeDistance = 3.0;
    public static final double kShooterStage2RPSLongRange = 120.0; // rot/s
    public static final double kShooterStage2RPSCap = 130.0; // rot/s
    public static final double kShooterStage1RPS = 70.0; // rot/s
    public static final double kShooterStage2Epsilon = 3.0;
    public static final double kShooterSpinupStage1RPS = 0.0;

    public static final double kShooterStage1IntakeRPS = 4.0;
    public static final double kShooterStage1ExhaustRPS = -10.0;

    public static final double kFenderShotRPS = 100.0;
    public static final double kPreloadShotRPS = 90.0;

    public static final double kBottomRollerSpeedupFactor = 1.0; // multiplied to all setpoints to determine how
                                                                 // much
                                                                 // extra power to give the bottom roller. >1.0 =
                                                                 // faster
                                                                 // bottom roller
    public static final double kTopRollerSpeedupFactor = 1.0;

    public static final double kBallReleaseHeight = Units.inchesToMeters(22.183);
}
