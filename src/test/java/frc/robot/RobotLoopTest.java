package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.logging.ComponentVisualizer;
import frc.robot.util.sim.FuelSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.state.SubsystemManagerFactory;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;

/**
 * Builds the real robot, runs its loop, and drives it through a shot.
 *
 * <p>Everything is in one instance because {@code AutoBuilder} and the subsystem manager are both
 * process wide singletons that can only be configured once.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class RobotLoopTest {
  private RobotState robotState;

  @BeforeAll
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    // Not Logger.start(): AdvantageKit exits the JVM unless the main class is a
    // LoggedRobot. Recording outputs without it is a no-op, which is all a test needs.

    // The command scheduler refuses to run anything while the robot reads as disabled.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    robotState = new RobotContainer().getRobotState();
    SubsystemManagerFactory.getInstance().registerSubsystem(robotState);
    SubsystemManagerFactory.getInstance().notifyTeleopStart();
  }

  @AfterAll
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Runs the scheduler the way {@code Robot.robotPeriodic} does. */
  private void runLoops(int count) {
    for (int i = 0; i < count; i++) {
      CommandScheduler.getInstance().run();
    }
  }

  @Test
  @Order(1)
  void theRobotLoopRunsWithoutBlowingUp() {
    runLoops(50);
    assertEquals(RobotState.State.ENABLED, robotState.getState());
    assertNotNull(robotState.getCurrentHubSetpoint());
  }

  @Test
  @Order(2)
  void theSolverFindsAShotFromAReasonableStartingSpot() {
    robotState.getDrive().setPose(new Pose2d(1.5, 4.02, Rotation2d.kZero));
    runLoops(5);

    assertTrue(
        robotState.getCurrentHubSetpoint().isAchievable(),
        "should have a shot from 3.1 m out in front of the hub");
    assertTrue(
        robotState.getCurrentHubSetpoint().getFlywheelSurfaceSpeed() > 1.0,
        "a live shot needs a real flywheel speed");
  }

  @Test
  @Order(3)
  void holdingTheShootButtonSpinsUpAndScores() {
    robotState.getDrive().setPose(new Pose2d(1.5, 4.02, Rotation2d.kZero));
    int before = robotState.getSimFuelCount();

    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.shoot(robotState));

    // Two seconds is plenty for the turret, hood and flywheel to settle and feed. Vision
    // keeps nudging the pose estimate, so readiness is sampled over the whole run rather
    // than checked on one arbitrary loop.
    boolean wasReady = false;
    for (int i = 0; i < 100; i++) {
      CommandScheduler.getInstance().run();
      wasReady |= robotState.getShooter().readyToShoot();
    }

    assertEquals(
        Superstructure.State.SHOOTING,
        robotState.getSuperstructure().getState(),
        "holding shoot should end up in SHOOTING, not fall back out of it");
    assertEquals(
        Shooter.State.SHOOTING,
        robotState.getShooter().getState(),
        "the shooter should have been cascaded into SHOOTING");
    assertTrue(wasReady, "turret, hood and flywheel should have come on target");
    assertTrue(
        robotState.getSimFuelCount() < before,
        "fuel should have been fired, count went " + before + " -> " + robotState.getSimFuelCount());
  }

  @Test
  @Order(4)
  void releasingTheButtonReturnsToIdle() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.idle(robotState));
    runLoops(20);

    assertEquals(Superstructure.State.IDLE, robotState.getSuperstructure().getState());
    assertEquals(Shooter.State.IDLE, robotState.getShooter().getState());
  }

  @Test
  @Order(5)
  void componentPosesAreDrawableAndFollowTheMechanisms() {
    robotState.getDrive().setPose(new Pose2d(1.5, 4.02, Rotation2d.kZero));
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.idle(robotState));
    runLoops(40);

    Pose3d[] poses = ComponentVisualizer.measured(robotState);
    assertEquals(7, poses.length, "one pose per model_INDEX.glb in the asset pack");

    for (int i = 0; i < poses.length; i++) {
      String name = ComponentVisualizer.kComponentNames[i];
      Pose3d pose = poses[i];
      assertTrue(
          Double.isFinite(pose.getX()) && Double.isFinite(pose.getY()) && Double.isFinite(pose.getZ()),
          name + " has a non-finite translation, AdvantageScope will not draw it");
      assertTrue(
          Double.isFinite(pose.getRotation().getY()),
          name + " has a non-finite rotation");
      // Robot relative and measured up from the floor, so everything belongs in a box
      // roughly the size of the robot.
      assertTrue(pose.getZ() > -0.3 && pose.getZ() < 2.0, name + " sits at an impossible height");
      assertTrue(
          pose.getTranslation().getNorm() < 2.0,
          name + " is further from the robot origin than the robot is big");
    }

    // Deploying the intake has to actually swing the intake component.
    double stowedPitch = poses[ComponentVisualizer.kIntake].getRotation().getY();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.intake(robotState));
    runLoops(60);

    Pose3d[] deployed = ComponentVisualizer.measured(robotState);
    assertTrue(
        Math.abs(deployed[ComponentVisualizer.kIntake].getRotation().getY() - stowedPitch) > 0.5,
        "the intake component should swing down when the intake deploys");

    // Static parts must not drift.
    assertEquals(
        poses[ComponentVisualizer.kBumpers],
        deployed[ComponentVisualizer.kBumpers],
        "the bumpers are static and should never move");
  }

  @Test
  @Order(7)
  void intakingPicksFuelOffTheFloorAndShootingFiresIt() {
    // Settle back to idle well away from any fuel first. Cancelling a command does not
    // change the subsystem's state, so the intake stays in INTAKE for a loop or two and
    // would hoover up the depot before the baseline is taken.
    CommandScheduler.getInstance().cancelAll();
    robotState.getDrive().setPose(new Pose2d(2.5, 4.02, Rotation2d.kZero));
    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.idle(robotState));
    runLoops(30);
    assertEquals(Intake.State.STOW, robotState.getIntake().getState(), "intake should be stowed");

    // Drop a ball just in front of the bumper rather than driving to the depot, so the
    // check does not depend on what earlier tests already swept up. Place it against the
    // pose the robot actually believes it is at: teleporting mid match leaves stale vision
    // measurements in the estimator's buffer that take a moment to wash out.
    Pose2d pose = robotState.getDrive().getPose();
    robotState
        .getFuelSim()
        .spawnFuel(
            new Translation3d(
                pose.getX() + 0.45 * pose.getRotation().getCos(),
                pose.getY() + 0.45 * pose.getRotation().getSin(),
                FuelSim.FUEL_RADIUS),
            new Translation3d());
    int before = robotState.getSimFuelCount();

    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.intake(robotState));
    runLoops(80);

    assertEquals(
        Intake.State.INTAKE,
        robotState.getIntake().getState(),
        "holding intake should put the intake in INTAKE, deployed with rollers running");
    assertTrue(
        Math.abs(robotState.getIntake().getRollerSpeed()) > 1.0,
        "the intake rollers should actually be turning, were "
            + robotState.getIntake().getRollerSpeed());
    assertTrue(
        robotState.getSimFuelCount() > before,
        "fuel should have been collected, count went "
            + before
            + " -> "
            + robotState.getSimFuelCount()
            + "; robot at "
            + robotState.getDrive().getPose()
            + "; fuel on field "
            + robotState.getFuelSim().getFuelCount()
            + "; nearest fuel rel "
            + nearestFuelRobotRelative(robotState));

    // Now carry it to a shooting spot and put it through the hub.
    int carried = robotState.getSimFuelCount();
    int scoreBefore = FuelSim.Hub.BLUE_HUB.getScore();

    CommandScheduler.getInstance().cancelAll();
    robotState.getDrive().setPose(new Pose2d(1.5, 4.02, Rotation2d.kZero));
    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.shoot(robotState));
    runLoops(150);

    assertTrue(
        robotState.getSimFuelCount() < carried,
        "shooting should consume the fuel that was collected");
    assertTrue(
        FuelSim.Hub.BLUE_HUB.getScore() > scoreBefore,
        "the collected fuel should end up in the hub");
  }

  private static String nearestFuelRobotRelative(RobotState state) {
    Pose2d robot = state.getDrive().getPose();
    double best = Double.MAX_VALUE;
    String out = "none";
    for (Translation3d f : state.getFuelSim().debugFuelPositions()) {
      var rel =
          new Pose2d(f.toTranslation2d(), Rotation2d.kZero).relativeTo(robot).getTranslation();
      if (rel.getNorm() < best) {
        best = rel.getNorm();
        out = String.format("x=%.3f y=%.3f z=%.3f", rel.getX(), rel.getY(), f.getZ());
      }
    }
    return out;
  }

  @Test
  @Order(8)
  void theRobotDoesNotWanderWhileDisabled() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().cancelAll();

    Pose2d start = new Pose2d(1.6, 4.02, Rotation2d.kZero);
    robotState.getDrive().setPose(start);

    runLoops(50); // let the estimator take its first look at the tags
    Pose2d a = robotState.getDrive().getPose();
    runLoops(150);
    Pose2d b = robotState.getDrive().getPose();
    runLoops(150);
    Pose2d c = robotState.getDrive().getPose();

    double firstWindow = b.getTranslation().getDistance(a.getTranslation());
    double secondWindow = c.getTranslation().getDistance(b.getTranslation());
    double total = c.getTranslation().getDistance(start.getTranslation());

    // The estimator settling onto a small constant vision offset is fine. What is not fine
    // is it walking: that was the vision sim rendering its tags from the estimator's own
    // output, so every solve re-observed and re-applied its own bias.
    assertTrue(total < 0.05, "disabled robot wandered " + total + " m from where it was placed");
    assertTrue(
        secondWindow < firstWindow || secondWindow < 0.002,
        "disabled pose is not settling: moved "
            + firstWindow
            + " m then "
            + secondWindow
            + " m over equal windows");
    assertEquals(
        0.0,
        robotState
            .getDrive()
            .getWheelOdometryPose()
            .getTranslation()
            .getDistance(start.getTranslation()),
        1e-9,
        "the wheels never turned, so the wheels-only pose must not have moved at all");

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  @Order(9)
  void theLoopKeepsUpWithRealTime() {
    runLoops(20); // warm up

    int loops = 100;
    long start = System.nanoTime();
    runLoops(loops);
    double millisPerLoop = (System.nanoTime() - start) / 1e6 / loops;

    assertTrue(
        millisPerLoop < 20.0,
        "robot loop took " + millisPerLoop + " ms, which overruns the 20 ms period");
  }
}
