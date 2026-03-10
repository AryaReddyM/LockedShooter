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

    public static final double kTurretP = 0.91;
    public static final double kTurretI = 0;
    public static final double kTurretD = 0;
    public static final double kTurretS = 0;
    public static final double kTurretV = 0;
    public static final double kTurretA = 0;
    public static final double kTurretG = 0;
    public static final double kTurretMaxAccel = 800;
    public static final double kTurretCruiseVel = 600;
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

    public static final double[] LOGGED_DISTANCES = new double[] {
        1.409409,
        1.822781,
        2.088259,
        2.338346,
        2.527475,
        2.787548,
        3.097655,
        3.59677,
        4.033556,
        4.611542,
        5.050931,
        5.525151,
        5.944625,
        6.546274
    };


        
        static {
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
        }

        public static final Distance kdistanceAboveFunnel = Inches.of(20);
        public static final Angle kMinTurnAngle = Radians.of(kBackwardSoftLimit);
        public static final Angle kMaxTurnAngle = Radians.of(kForwardSoftLimit);
    
}