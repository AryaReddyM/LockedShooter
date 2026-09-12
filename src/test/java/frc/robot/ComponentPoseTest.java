package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.logging.ComponentVisualizer;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Checks the component poses on their own, without needing a whole robot. The home poses are
 * solved from the asset's config.json, so these guard the arithmetic rather than the robot.
 */
class ComponentPoseTest {
  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
  }

  @Test
  void thereIsOnePoseForEveryModelInTheAsset() {
    assertEquals(7, ComponentVisualizer.kComponentNames.length);
    assertEquals(7, ComponentVisualizer.home().length);
    assertEquals(7, ComponentVisualizer.identify().length);
  }

  @Test
  void homePosesAreFiniteAndOnTheRobot() {
    Pose3d[] home = ComponentVisualizer.home();
    for (int i = 0; i < home.length; i++) {
      String name = ComponentVisualizer.kComponentNames[i];
      Pose3d pose = home[i];
      assertTrue(
          Double.isFinite(pose.getX()) && Double.isFinite(pose.getY()) && Double.isFinite(pose.getZ()),
          name + " home pose is not finite");
      assertTrue(
          pose.getTranslation().getNorm() < 1.0,
          name + " home pose is " + pose.getTranslation().getNorm() + " m from the origin");
    }
  }

  @Test
  void homeIsNotAccidentallyAllIdentity() {
    // If the solve collapsed to identity the components would all stack on the origin,
    // which is exactly the failure mode this replaced.
    long nonIdentity =
        java.util.Arrays.stream(ComponentVisualizer.home())
            .filter(p -> !p.equals(Pose3d.kZero))
            .count();
    assertTrue(nonIdentity >= 5, "expected most components to sit away from the origin");
  }

  @Test
  void identifySpreadsTheModelsOut() {
    Pose3d[] spread = ComponentVisualizer.identify();
    for (int i = 1; i < spread.length; i++) {
      assertNotEquals(spread[i - 1].getX(), spread[i].getX());
      assertTrue(spread[i].getX() > spread[i - 1].getX());
    }
  }
}
