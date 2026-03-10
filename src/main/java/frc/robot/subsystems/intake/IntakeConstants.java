package frc.robot.subsystems.intake;

import org.dyn4j.geometry.Transform;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {

    
    public static final int kRollersCanID = 48;
    public static final int kExtensionCanID = 46;
    public static final int kExtensionFollowerCanID = 47;

    // Roller PID
    public static final double kRollerP = 0.022;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;

    public static final double kRollerSimP = 0;
    public static final double kRollerSimD = 0;

    // Extension PID

    public static final double kExtensionP = 0.09;
    public static final double kExtensionI = 0;
    public static final double kExtensionD = 0;
    public static final double kExtensionMaxAccel = 600;
    public static final double kExtensionCruiseVel = 130;
    public static final double kExtensionDeviationErr = 2;
    public static final double kExtensionS = 0.17;
    public static final double kExtensionV = 0.00131;
    public static final double kExtensionA = 0;
    public static final double kExtensionCos = 0.24;

    public static final double kExtensionSimP = 0.3;
    public static final double kExtensionSimD = 0;


    // factors
    public static final double kRollerPositionConversionFactor = 1.0;
    public static final double kRollerVelocityConversionFactor = 1.0/60.0;

    public static int kRollerCurrentLimit = 70;

    public static final double kExtensionPositionConversionFactor = 360.0/23.0;
    public static final double kExtensionVelocityConversionFactor = (360.0/23.0) / 60.0;

    public static int kExtensionCurrentLimit = 60;
    // setpoints

    public static final double kExtensionStowSetpoint = -93;
    public static final double kExtensionIntakeSetpoint = 0;
    public static final double kExtensionOuttakeSetpoint = 0;
    public static final double kExtensionClimbTowSetpoint = -50;
    public static final double kExtensionShakeSetpoint = -60;

    public static final double kExtensionMax = 0;
    public static final double kExtensionMin = -96;

    public static final double kRollerOuttakeSpeed = 40;
    public static final double kRollerIntakeSpeed = -40;

    public static final Transform3d intakeOrigin = new Transform3d(new Translation3d(
        Units.inchesToMeters(5),Units.inchesToMeters(0), Units.inchesToMeters(6.7)
    ), new Rotation3d(0, Units.degreesToRadians(0), 0));
}