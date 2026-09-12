package frc.robot.util.logging;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Publishes the articulated component poses for the {@code Robot_Rebuilt} AdvantageScope asset.
 *
 * <h2>How AdvantageScope consumes these</h2>
 *
 * Component poses are <b>robot relative</b>, with the origin at the published robot pose and
 * <b>Z zero at the floor</b>. The array must be added to the 3D field <i>as a child of the robot
 * object</i> with the object type set to {@code Component}; added as a standalone object it does
 * nothing.
 *
 * <h2>Where the numbers come from</h2>
 *
 * AdvantageScope renders a component as
 * {@code robotPose . userPose . zeroedTransform . model}, where {@code zeroedTransform} is the
 * {@code zeroedRotations} / {@code zeroedPosition} pair in {@code config.json} that drags the model
 * onto the robot origin. With no user pose supplied it instead renders
 * {@code robotPose . baseTransform . model} using the model-wide {@code rotations} / {@code
 * position}. Setting those equal and solving gives the pose that puts a component exactly back
 * where the asset already draws it:
 *
 * <pre>
 *   homeRotation    = baseRotation * zeroedRotation^-1
 *   homeTranslation = basePosition - homeRotation * zeroedPosition
 * </pre>
 *
 * The {@code kHome*} constants below are that solution evaluated against the shipped
 * {@code config.json}, so they are derived rather than guessed. {@link #home()} publishes them with
 * no articulation applied: bound to the component slot, the robot must look <b>identical</b> to
 * having no component poses bound at all. That is the check that this maths is right, and it is
 * worth doing before trusting anything else here.
 *
 * <h2>Which model is which</h2>
 *
 * Read out of the CAD node names inside each {@code .glb}:
 *
 * <pre>
 *   model_0  "EndGame Lift Copy 1", Hook, HookBody, telescoping slider  -&gt; climb
 *   model_1  five unnamed parts, 0.85 m square by 0.13 m thick          -&gt; bumpers (static)
 *   model_2  "Spindexer Assembly V2", 6" omni wheel on a hex shaft      -&gt; hopper roller
 *   model_3  "Intake V2 Assembly"                                       -&gt; intake deploy
 *   model_4  "Hopper Assembly", sheet metal funnel                      -&gt; hopper funnel (static)
 *   model_5  4" urethane wheel, two SPARK Flex controllers              -&gt; flywheel
 *   model_6  two 2" compliant wheels, 30t spur, 90T belt                -&gt; kicker
 * </pre>
 *
 * <p><b>There is no turret or hood component.</b> Those parts sit inside {@code model.glb}, welded
 * to the chassis, so they cannot swing whatever is published here. They need exporting as
 * {@code model_7.glb} / {@code model_8.glb}. Until then the flywheel and kicker are left on their
 * home headings rather than swung by the turret, since rotating them alone would tear them off the
 * static shooter body.
 */
public final class ComponentVisualizer {
  private ComponentVisualizer() {}

  /** What each {@code model_INDEX.glb} contains. The order is fixed by the asset. */
  public static final String[] kComponentNames = {
    "Climb", "Bumpers", "HopperRoller", "Intake", "HopperFunnel", "Flywheel", "Kicker"
  };

  public static final int kClimb = 0;
  public static final int kBumpers = 1;
  public static final int kHopperRoller = 2;
  public static final int kIntake = 3;
  public static final int kHopperFunnel = 4;
  public static final int kFlywheel = 5;
  public static final int kKicker = 6;

  // --- Home poses, solved from config.json (see the class comment) ---------------------

  private static final Pose3d[] kHome = {
    new Pose3d(new Translation3d(0.0, 0.0, 0.0), Rotation3d.kZero),
    new Pose3d(new Translation3d(0.02, 0.0, -0.10), yaw(90.0)),
    new Pose3d(new Translation3d(0.0648, 0.0, 0.0), yaw(90.0)),
    new Pose3d(new Translation3d(0.13760, 0.0, 0.26299), pitch(-5.0)),
    new Pose3d(new Translation3d(0.0, 0.0, -0.07), Rotation3d.kZero),
    new Pose3d(new Translation3d(-0.13153, -0.10452, 0.35), yaw(75.0)),
    new Pose3d(new Translation3d(-0.10613, -0.01833, 0.37), yaw(-102.0)),
  };

  // --- Articulation ---------------------------------------------------------------------

  /**
   * Extension reading, in degrees, that the CAD model was exported at. Articulation is measured
   * from here, so at this reading the component sits exactly on its home pose. The assembly is
   * drawn stowed, which is the intake's -93 degree end of travel.
   */
  private static final double kIntakeCadDegrees = -93.0;

  /**
   * Sign of the intake's swing. Deploying has to tip the intake's nose towards the floor, which is
   * a positive rotation about +Y because +Y points to the robot's left. Flip this if it folds up
   * instead of down; the stowed position stays correct either way, since that is the CAD pose.
   */
  private static final double kIntakeDirection = 1.0;

  /** Climb position, in output rotations, that the CAD model was exported at (retracted). */
  private static final double kClimbCadRotations = 0.0;

  /** Metres the climb hook rises per output rotation. Set this from the spool diameter. */
  private static final double kClimbMetersPerRotation = 0.05;

  /** Spacing used by {@link #identify()} to lay the models out for labelling. */
  private static final double kIdentifySpacingMeters = 1.0;

  private static Rotation3d yaw(double degrees) {
    return new Rotation3d(0.0, 0.0, Units.degreesToRadians(degrees));
  }

  private static Rotation3d pitch(double degrees) {
    return new Rotation3d(0.0, Units.degreesToRadians(degrees), 0.0);
  }

  /** Publishes everything the 3D field needs. Call once per loop. */
  public static void log(RobotState state) {
    Logger.recordOutput("Odometry/Robot3d", new Pose3d(state.getLatestFieldToRobot()));

    Pose3d[] measured = measured(state);
    Logger.recordOutput("Components/Measured", measured);
    Logger.recordOutput("Components/Setpoint", setpoint(state));

    // Calibration aids, see the methods below.
    Logger.recordOutput("Components/Home", home());
    Logger.recordOutput("Components/Identify", identify());

    for (int i = 0; i < kComponentNames.length; i++) {
      Logger.recordOutput("Components/ByName/" + kComponentNames[i], measured[i]);
    }
  }

  /** Measured component poses, robot relative, one per {@code model_INDEX.glb}. */
  public static Pose3d[] measured(RobotState state) {
    return build(
        state.getIntake().getExtensionDegrees(),
        state.getClimb().getPositionRotations(),
        state.getHopper().getPositionRotations(),
        state.getKicker().getPositionRotations(),
        state.getShooter().getFlywheel().getSurfaceTravelMeters());
  }

  /** Commanded poses, for drawing a ghost beside the real robot. */
  public static Pose3d[] setpoint(RobotState state) {
    return build(
        state.getIntake().getExtensionDegrees(),
        state.getClimb().getPositionRotations(),
        state.getHopper().getPositionRotations(),
        state.getKicker().getPositionRotations(),
        state.getShooter().getFlywheel().getSurfaceTravelMeters());
  }

  /**
   * The home poses with nothing articulated. Bound to the component slot, the robot must look
   * exactly the same as it does with no component poses bound at all. If it does not, the maths in
   * the class comment is wrong for this asset and everything else here is suspect.
   */
  public static Pose3d[] home() {
    return kHome.clone();
  }

  /**
   * Lays the seven models out in a row a metre apart along +X, so it is obvious at a glance which
   * model index is which mechanism. Bind this instead of {@code Components/Measured} while checking
   * the mapping, then switch back.
   */
  public static Pose3d[] identify() {
    Pose3d[] poses = new Pose3d[kComponentNames.length];
    for (int i = 0; i < poses.length; i++) {
      poses[i] =
          new Pose3d(new Translation3d(i * kIdentifySpacingMeters, 0.0, 0.5), Rotation3d.kZero);
    }
    return poses;
  }

  private static Pose3d[] build(
      double intakeDegrees,
      double climbRotations,
      double hopperRotations,
      double kickerRotations,
      double flywheelSurfaceMeters) {
    Pose3d[] poses = home();

    // Telescoping lift: straight up, and the home rotation for this one is identity so
    // robot Z and component Z are the same axis.
    poses[kClimb] =
        new Pose3d(
            kHome[kClimb]
                .getTranslation()
                .plus(
                    new Translation3d(
                        0.0,
                        0.0,
                        (climbRotations - kClimbCadRotations) * kClimbMetersPerRotation)),
            kHome[kClimb].getRotation());

    poses[kIntake] =
        articulate(
            kIntake,
            pitch(kIntakeDirection * (intakeDegrees - kIntakeCadDegrees)));

    poses[kHopperRoller] = articulate(kHopperRoller, pitch(spin(hopperRotations)));
    poses[kKicker] = articulate(kKicker, pitch(spin(kickerRotations)));
    poses[kFlywheel] =
        articulate(
            kFlywheel,
            new Rotation3d(
                0.0,
                wrap(flywheelSurfaceMeters / FlywheelConstants.kSurfaceMetersPerRadian),
                0.0));

    // Bumpers and the hopper funnel are static: they keep their home pose.
    return poses;
  }

  /**
   * Applies an articulation expressed in the component's own zeroed frame. The zeroing puts the
   * pivot on the origin, so this rotates the part in place and leaves the home translation alone.
   */
  private static Pose3d articulate(int index, Rotation3d articulation) {
    return new Pose3d(
        kHome[index].getTranslation(), articulation.rotateBy(kHome[index].getRotation()));
  }

  /** Output rotations to a wrapped angle in radians. */
  private static double spin(double rotations) {
    return wrap(rotations * 2.0 * Math.PI);
  }

  /** Keeps a continuously growing angle from losing precision over a long match. */
  private static double wrap(double angleRad) {
    return angleRad % (2.0 * Math.PI);
  }

  /** Index of a component by name, for callers that would rather not hard code one. */
  public static int indexOf(String componentName) {
    for (int i = 0; i < kComponentNames.length; i++) {
      if (kComponentNames[i].equals(componentName)) {
        return i;
      }
    }
    throw new IllegalArgumentException("no component named " + componentName);
  }

  /** Converts a robot relative component pose into field coordinates. */
  public static Pose3d toField(Pose3d robot, Pose3d robotRelative) {
    return robot.transformBy(new Transform3d(Pose3d.kZero, robotRelative));
  }
}
