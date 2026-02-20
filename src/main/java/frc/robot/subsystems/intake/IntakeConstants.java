package frc.robot.subsystems.intake;

import org.dyn4j.geometry.Transform;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {

    
    public static final int kRollersCanID = 0;
    public static final int kExtensionCanID = 0;

    // Roller PID
    public static final double kRollerP = 0;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;

    public static final double kRollerSimP = 0;
    public static final double kRollerSimD = 0;

    // Extension PID

    public static final double kExtensionP = 0;
    public static final double kExtensionI = 0;
    public static final double kExtensionD = 0;
    public static final double kExtensionMaxAccel = 0;
    public static final double kExtensionCruiseVel = 0;
    public static final double kExtensionDeviationErr = 0;

    public static final double kExtensionSimP = 0.3;
    public static final double kExtensionSimD = 0;


    // factors
    public static final double kRollerPositionConversionFactor = 0;
    public static final double kRollerVelocityConversionFactor = 0;

    public static final double kExtensionPositionConversionFactor = 0;
    public static final double kExtensionVelocityConversionFactor = 0;

    // setpoints
    public static final double kExtensionStowSetpoint = -0.1;
    public static final double kExtensionIntakeSetpoint = -1;
    public static final double kExtensionOuttakeSetpoint = -1;
    public static final double kRollerOuttakeSpeed = 0;
    public static final double kRollerIntakeSpeed = 0;

    public static final Transform3d intakeOrigin = new Transform3d(new Translation3d(
        Units.inchesToMeters(10.360),Units.inchesToMeters(0), Units.inchesToMeters(0)
    ), new Rotation3d(0, Units.degreesToRadians(0), 0));
}
