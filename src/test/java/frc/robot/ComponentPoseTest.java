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
    // The asset ships model_0.glb through model_6.glb, so exactly seven poses may be
    // published. Turret and hood have slots reserved but no models yet.
    assertEquals(7, ComponentVisualizer.kPublishedComponents);
    assertEquals(
        ComponentVisualizer.kPublishedComponents,
        ComponentVisualizer.home().length,
        "home() must not hand AdvantageScope more poses than the asset has models");
    assertEquals(
        ComponentVisualizer.kPublishedComponents,
        ComponentVisualizer.identify().length,
        "identify() must match the published component count too");
    assertTrue(
        ComponentVisualizer.kComponentNames.length >= ComponentVisualizer.kPublishedComponents,
        "every published index needs a name");
  }

  @Test
  void turretAndHoodHaveSlotsReservedButAreNotPublishedYet() {
    assertEquals(7, ComponentVisualizer.kTurret);
    assertEquals(8, ComponentVisualizer.kHood);
    assertEquals("Turret", ComponentVisualizer.kComponentNames[ComponentVisualizer.kTurret]);
    assertEquals("Hood", ComponentVisualizer.kComponentNames[ComponentVisualizer.kHood]);
    assertTrue(
        ComponentVisualizer.kTurret >= ComponentVisualizer.kPublishedComponents,
        "turret must stay unpublished until model_7.glb exists");
    assertTrue(
        ComponentVisualizer.kHood >= ComponentVisualizer.kPublishedComponents,
        "hood must stay unpublished until model_8.glb exists");
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
