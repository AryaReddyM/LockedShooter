package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.TurretCalculator.ShotData;

// Turret PID
public class TurretConstants {
    public static final double kTurretP = 0;
    public static final double kTurretI = 0;
    public static final double kTurretD = 0;
    public static final double kTurretS = 0;
    public static final double kTurretV = 0;
    public static final double kTurretA = 0;
    public static final double kTurretG = 0;
    public static final double kTurretMaxAccel = 0;
    public static final double kTurretCruiseVel = 0;
    public static final double kTurretDeviationErr = 0;

    public static final Rotation2d kTurretAbsEncoderOffset = Rotation2d.fromRadians(0);

    public static final double kTurretSimP = 0.1;
    public static final double kTurretSimD = 0;



    // factors
    public static final double kTurretPositionConversionFactor = 2.0 * Math.PI / 4.0 / (200.0 / 20.0);
    public static final double kTurretVelocityConversionFactor = (2.0 * Math.PI / 4.0 / (200.0 / 20.0)) / 60;


    // Configuration
    public static final boolean kTurretinverted = false;
    public static final int kTurretCurrentLimit = 10;


    public static final double latencyComepnsationMS = 20.0;

    public static final double kMinOutputRange = 0;
    public static final double kMaxOutputRange = 2 * Math.PI;
    public static final double kForwardSoftLimit = Math.PI;
    public static final double kBackwardSoftLimit = -Math.PI;
    
    public static final InterpolatingTreeMap<Double, ShotData> SHOT_MAP =
                new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

        public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();



        static {
            SHOT_MAP.put(5.34, new ShotData(RPM.of(2900), Degrees.of(27)));
            TOF_MAP.put(5.34, 1.30);

            SHOT_MAP.put(4.90, new ShotData(RPM.of(2700), Degrees.of(26)));
            TOF_MAP.put(4.90, 1.42);

            SHOT_MAP.put(4.44, new ShotData(RPM.of(2820), Degrees.of(25.5)));
            TOF_MAP.put(4.44, 1.34);

            SHOT_MAP.put(4.05, new ShotData(RPM.of(2800), Degrees.of(25)));
            TOF_MAP.put(4.05, 1.36);

            SHOT_MAP.put(3.74, new ShotData(RPM.of(2750), Degrees.of(24)));
            TOF_MAP.put(3.74, 1.21);

            SHOT_MAP.put(3.42, new ShotData(RPM.of(2700), Degrees.of(23)));
            TOF_MAP.put(3.42, 1.40);

            SHOT_MAP.put(3.06, new ShotData(RPM.of(2610), Degrees.of(22)));
            TOF_MAP.put(3.06, 1.38);

            SHOT_MAP.put(2.73, new ShotData(RPM.of(2500), Degrees.of(20.5)));
            TOF_MAP.put(2.73, 1.34);

            SHOT_MAP.put(2.45, new ShotData(RPM.of(2450), Degrees.of(19.5)));
            TOF_MAP.put(2.45, 1.28);

            SHOT_MAP.put(2.14, new ShotData(RPM.of(2400), Degrees.of(18)));
            TOF_MAP.put(2.14, 1.31);

            SHOT_MAP.put(1.86, new ShotData(RPM.of(2350), Degrees.of(17)));
            TOF_MAP.put(1.86, 1.24);

            SHOT_MAP.put(1.55, new ShotData(RPM.of(2275), Degrees.of(15)));
            TOF_MAP.put(1.55, 1.23);
        }

        public static final Distance kdistanceAboveFunnel = Inches.of(20);
        public static final Angle kMinTurnAngle = Rotations.of(-0.55);
        public static final Angle kMaxTurnAngle = Rotations.of(0.55);
    
}