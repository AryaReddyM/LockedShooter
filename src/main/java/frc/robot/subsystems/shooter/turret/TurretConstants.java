package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.util.TurretCalculator;
import frc.robot.util.TurretCalculator.ShotData;

// Turret PID
public class TurretConstants {
    public static final int kTurretCanId = 24;

    public static final double kTurretP = 0.9;
    public static final double kTurretI = 0;
    public static final double kTurretD = 0;
    public static final double kTurretS = 0;
    public static final double kTurretV = 0;
    public static final double kTurretA = 0;
    public static final double kTurretG = 0;
    public static final double kTurretMaxAccel = 600;
    public static final double kTurretCruiseVel = 300;
    public static final double kTurretDeviationErr = 1;

    public static final Rotation2d kTurretAbsEncoderOffset = Rotation2d.fromRadians(0);

    public static final double kTurretSimP = 0.1;
    public static final double kTurretSimD = 0;



    // factors
    public static final double kTurretPositionConversionFactor = 2.0 * Math.PI / 4.0 / (200.0 / 20.0);
    public static final double kTurretVelocityConversionFactor = (2.0 * Math.PI / 4.0 / (200.0 / 20.0)) / 60;


    public static final double kReadyToleranceDegrees = 4.0;

    // Configuration
    public static final boolean kTurretinverted = true;
    public static final int kTurretCurrentLimit = 20;


    public static final double latencyComepnsationMS = 20.0;

    public static final double kForwardSoftLimit = 2.7;//Math.PI;
    public static final double kBackwardSoftLimit = -Math.PI - (Math.PI  - kForwardSoftLimit);
    
    public static final InterpolatingTreeMap<Double, ShotData> SHOT_MAP =
                new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

    public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();



        
        static {
            // SHOT_MAP.put(5.34, new ShotData(RPM.of(2900), Degrees.of(27)));
            // TOF_MAP.put(5.34, 1.30);

            // SHOT_MAP.put(4.90, new ShotData(RPM.of(2700), Degrees.of(26)));
            // TOF_MAP.put(4.90, 1.42);

            // SHOT_MAP.put(4.44, new ShotData(RPM.of(2820), Degrees.of(25.5)));
            // TOF_MAP.put(4.44, 1.34);

            // SHOT_MAP.put(4.05, new ShotData(RPM.of(2800), Degrees.of(25)));
            // TOF_MAP.put(4.05, 1.36);

            // SHOT_MAP.put(3.74, new ShotData(RPM.of(2750), Degrees.of(24)));
            // TOF_MAP.put(3.74, 1.21);

            // SHOT_MAP.put(3.42, new ShotData(RPM.of(2700), Degrees.of(23)));
            // TOF_MAP.put(3.42, 1.40);

            // SHOT_MAP.put(3.06, new ShotData(RPM.of(2610), Degrees.of(22)));
            // TOF_MAP.put(3.06, 1.38);

            // SHOT_MAP.put(2.73, new ShotData(RPM.of(2500), Degrees.of(20.5)));
            // TOF_MAP.put(2.73, 1.34);

            // SHOT_MAP.put(2.45, new ShotData(RPM.of(2450), Degrees.of(19.5)));
            // TOF_MAP.put(2.45, 1.28);

            // SHOT_MAP.put(2.14, new ShotData(RPM.of(2400), Degrees.of(18)));
            // TOF_MAP.put(2.14, 1.31);

            // SHOT_MAP.put(1.86, new ShotData(RPM.of(2350), Degrees.of(17)));
            // TOF_MAP.put(1.86, 1.24);

            // SHOT_MAP.put(1.55, new ShotData(RPM.of(2275), Degrees.of(15)));
            // TOF_MAP.put(1.55, 1.23);

            // SHOT_MAP.put(2.947297, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(21, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radians.of(0.431984)));
            // TOF_MAP.put(2.947297, 1.4);

            // SHOT_MAP.put(3.409922,new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(21.2, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radians.of(0.492857)));
            // TOF_MAP.put(0.492857, 1.3);

            // SHOT_MAP.put(4.310985, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(22.5, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radians.of(0.562425)));
            // TOF_MAP.put(4.310985, 1.42);

            // SHOT_MAP.put(distance, RadiansPerSecond.of(Flywheel.omegaRadPerSec), Radians.of(hood.pos));
            // TOF_MAP.put(distance, amount in air before it lands at expected target);


            SHOT_MAP.put(1.409409, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(8.7, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0)));
            TOF_MAP.put(1.409409, 0.6);

            SHOT_MAP.put(1.822781, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(9.8, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0)));
            TOF_MAP.put(1.822781, 0.9);

            SHOT_MAP.put(2.088259, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(9.96, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0)));
            TOF_MAP.put(2.088259, 1.1);


            SHOT_MAP.put(2.338346, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(10.4, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0)));
            TOF_MAP.put(2.338346, 1.2);

            SHOT_MAP.put(2.527475, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(10.8, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0)));
            TOF_MAP.put(2.527475, 1.25);

            SHOT_MAP.put(2.787548, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(10.6, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.5)));
            TOF_MAP.put(2.787548, 1.2);


            SHOT_MAP.put(3.097655, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(10.6, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.54)));
            TOF_MAP.put(3.097655, 1.17);

            SHOT_MAP.put(3.59677, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(11.05, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.59)));
            TOF_MAP.put(3.59677, 1.19);


            SHOT_MAP.put(3.59677, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(11.4, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.62)));
            TOF_MAP.put(3.59677, 1.0);


            SHOT_MAP.put(4.033556, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(11.2, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.65)));
            TOF_MAP.put(4.033556, 1.04);

            SHOT_MAP.put(4.611542, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(11.7, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.64)));
            TOF_MAP.put(4.611542, 1.1);

            SHOT_MAP.put(5.050931, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(12, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.66)));
            TOF_MAP.put(5.050931, 1.18);

            SHOT_MAP.put(5.525151, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(12.35, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.68)));
            TOF_MAP.put(5.525151, 1.2);

            
            SHOT_MAP.put(5.944625, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(12.9, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.7)));
            TOF_MAP.put(5.944625, 1.23);

            
            SHOT_MAP.put(6.546274, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(13.55, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.78)));
            TOF_MAP.put(6.546274, 1.4);
            // SHOT_MAP.put(2.803142, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(11.2, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0)));
            // TOF_MAP.put(2.803142, 1.25);

            // SHOT_MAP.put(3.172004, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(11.1, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.4)));
            // TOF_MAP.put(3.172004, 1.15);

            // SHOT_MAP.put(Units.inchesToMeters(71), new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(9.85, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Degrees.of(25)));
            // TOF_MAP.put(Units.inchesToMeters(71), 1.2);

            // SHOT_MAP.put(Units.inchesToMeters(32+47.5),new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(10.25, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Degrees.of(25)));
            // TOF_MAP.put(Units.inchesToMeters(32+47.5), 1.25);

            // SHOT_MAP.put(Units.inchesToMeters(32+58.5),new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(10.45, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Degrees.of(25)));
            // TOF_MAP.put(Units.inchesToMeters(32+58.5), 1.25);

            // SHOT_MAP.put(3.556634, new ShotData(TurretCalculator.linearToAngularVelocity(LinearVelocity.ofBaseUnits(10.85, MetersPerSecond), FlywheelConstants.kFlywheelRadius), Radian.of(0.6)));
            // TOF_MAP.put(3.556634, 1.3);
        }

        public static final Distance kdistanceAboveFunnel = Inches.of(20);
        public static final Angle kMinTurnAngle = Radians.of(kBackwardSoftLimit);
        public static final Angle kMaxTurnAngle = Radians.of(kForwardSoftLimit);
    
}