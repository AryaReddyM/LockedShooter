package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.sim.FuelSim;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Checks the parts of the fuel simulation the robot code drives. */
class FuelSimTest {
  private static final double kIntakeMinX = 0.25;
  private static final double kIntakeMaxX = 0.62;
  private static final double kIntakeHalfWidth = 0.33;

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
  }

  private static FuelSim simAt(Pose2d pose) {
    FuelSim sim = new FuelSim();
    sim.registerRobot(
        DriveConstants.trackWidth,
        DriveConstants.wheelBase,
        DriveConstants.kBumperHeight,
        () -> pose,
        ChassisSpeeds::new);
    sim.start();
    return sim;
  }

  @Test
  void anEnabledIntakePicksUpFuelInFrontOfTheRobot() {
    Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.kZero);
    FuelSim sim = simAt(pose);

    AtomicInteger collected = new AtomicInteger();
    sim.registerIntake(
        kIntakeMinX,
        kIntakeMaxX,
        -kIntakeHalfWidth,
        kIntakeHalfWidth,
        () -> true,
        collected::incrementAndGet);

    sim.spawnFuel(
        new Translation3d(pose.getX() + 0.45, pose.getY(), FuelSim.FUEL_RADIUS),
        new Translation3d());
    sim.updateSim();

    assertEquals(1, collected.get(), "fuel sitting in the intake should have been picked up");
  }

  @Test
  void aDisabledIntakeLeavesFuelAlone() {
    Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.kZero);
    FuelSim sim = simAt(pose);

    AtomicBoolean running = new AtomicBoolean(false);
    AtomicInteger collected = new AtomicInteger();
    sim.registerIntake(
        kIntakeMinX,
        kIntakeMaxX,
        -kIntakeHalfWidth,
        kIntakeHalfWidth,
        running::get,
        collected::incrementAndGet);

    sim.spawnFuel(
        new Translation3d(pose.getX() + 0.45, pose.getY(), FuelSim.FUEL_RADIUS),
        new Translation3d());

    for (int i = 0; i < 25; i++) {
      sim.updateSim();
    }
    assertEquals(0, collected.get(), "the intake was off, nothing should have been collected");

    running.set(true);
    sim.updateSim();
    assertEquals(1, collected.get(), "switching the intake on should collect the waiting fuel");
  }

  @Test
  void fuelBehindTheRobotIsNotCollected() {
    Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.kZero);
    FuelSim sim = simAt(pose);

    AtomicInteger collected = new AtomicInteger();
    sim.registerIntake(
        kIntakeMinX,
        kIntakeMaxX,
        -kIntakeHalfWidth,
        kIntakeHalfWidth,
        () -> true,
        collected::incrementAndGet);

    sim.spawnFuel(
        new Translation3d(pose.getX() - 0.45, pose.getY(), FuelSim.FUEL_RADIUS),
        new Translation3d());
    for (int i = 0; i < 10; i++) {
      sim.updateSim();
    }
    assertEquals(0, collected.get(), "the intake is only on the front of the robot");
  }

  @Test
  void aFullFieldOfFuelStepsFasterThanRealTime() {
    Pose2d pose = new Pose2d(2.0, 4.0, Rotation2d.kZero);
    FuelSim sim = simAt(pose);
    sim.enableAirResistance();
    sim.spawnStartingFuel();

    // Warm up, so JIT compilation is not counted as physics time.
    for (int i = 0; i < 50; i++) {
      sim.updateSim();
    }

    int loops = 100;
    long start = System.nanoTime();
    for (int i = 0; i < loops; i++) {
      sim.updateSim();
    }
    double millisPerLoop = (System.nanoTime() - start) / 1e6 / loops;

    // A robot loop is 20 ms and the fuel sim is only one of the things in it.
    assertTrue(
        millisPerLoop < 10.0,
        "fuel physics took " + millisPerLoop + " ms per loop, which will overrun the robot loop");
  }
}
