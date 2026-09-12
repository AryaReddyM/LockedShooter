package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Checks a simulated swerve module tracks what it is told.
 *
 * <p>The drive feedforward used to be about 23x too large, which pinned the module at 12 V for any
 * setpoint over about 4 rad/s. Everything still "worked" in the sense that the robot moved, it just
 * moved flat out no matter where the stick was, so these assertions are about partial speeds as
 * much as top speed.
 */
class DriveSimTest {
  private static final int kSettleLoops = 150;

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
  }

  private static ModuleIOInputsAutoLogged settleAtSpeed(ModuleIO io, double wheelRadPerSec) {
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    for (int i = 0; i < kSettleLoops; i++) {
      io.setDriveVelocity(wheelRadPerSec);
      io.setTurnPosition(Rotation2d.kZero);
      io.updateInputs(inputs);
    }
    return inputs;
  }

  private static double wheelRadPerSecFor(double metersPerSec) {
    return metersPerSec / DriveConstants.wheelRadiusMeters;
  }

  @Test
  void moduleTracksAHalfSpeedCommand() {
    ModuleIOInputsAutoLogged inputs = settleAtSpeed(new ModuleIOSim(), wheelRadPerSecFor(2.0));
    assertEquals(
        2.0,
        inputs.driveVelocityRadPerSec * DriveConstants.wheelRadiusMeters,
        0.1,
        "module should hold 2 m/s");
    assertTrue(
        Math.abs(inputs.driveAppliedVolts) < 11.5,
        "half speed should not need full voltage, was " + inputs.driveAppliedVolts + " V");
  }

  @Test
  void moduleTracksACrawl() {
    ModuleIOInputsAutoLogged inputs = settleAtSpeed(new ModuleIOSim(), wheelRadPerSecFor(0.4));
    assertEquals(
        0.4,
        inputs.driveVelocityRadPerSec * DriveConstants.wheelRadiusMeters,
        0.05,
        "module should hold a slow crawl rather than running away");
  }

  @Test
  void commandedSpeedsComeOutInOrder() {
    double previous = -1.0;
    for (double target : new double[] {0.5, 1.0, 2.0, 3.0, 4.0}) {
      ModuleIOInputsAutoLogged inputs = settleAtSpeed(new ModuleIOSim(), wheelRadPerSecFor(target));
      double actual = inputs.driveVelocityRadPerSec * DriveConstants.wheelRadiusMeters;
      assertTrue(actual > previous, "faster commands should give faster wheels, at " + target);
      previous = actual;
    }
  }

  @Test
  void moduleTakesARealisticTimeToGetUpToSpeed() {
    ModuleIO io = new ModuleIOSim();
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    double target = wheelRadPerSecFor(4.0);

    int loops = 0;
    while (loops < kSettleLoops && inputs.driveVelocityRadPerSec < target * 0.9) {
      io.setDriveVelocity(target);
      io.updateInputs(inputs);
      loops++;
    }
    // Anything under a loop or two means the sim has no meaningful chassis inertia and
    // driving feels like teleporting.
    assertTrue(loops >= 3, "spin up was instant (" + loops + " loops), inertia is unrealistic");
    assertTrue(loops < kSettleLoops, "module never reached 90% of 4 m/s");
  }

  @Test
  void steeringReachesCommandedAngles() {
    for (double degrees : new double[] {45, -90, 179}) {
      ModuleIO io = new ModuleIOSim();
      ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
      Rotation2d target = Rotation2d.fromDegrees(degrees);
      for (int i = 0; i < kSettleLoops; i++) {
        io.setTurnPosition(target);
        io.updateInputs(inputs);
      }
      assertEquals(
          0.0,
          inputs.turnPosition.minus(target).getRadians(),
          0.02,
          "steering should reach " + degrees + " degrees");
    }
  }
}
