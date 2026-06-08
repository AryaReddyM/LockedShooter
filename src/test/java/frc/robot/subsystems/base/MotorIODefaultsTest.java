package frc.robot.subsystems.base;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Verifies the feedforward overloads fall back to the plain methods (the replay/partial contract). */
class MotorIODefaultsTest {

  @Test
  void velocityFeedforwardOverloadDelegatesToPlain() {
    final double[] got = {Double.NaN};
    MotorIO io =
        new MotorIO() {
          @Override
          public void setVelocity(double velocityRadPerSec) {
            got[0] = velocityRadPerSec;
          }
        };

    // An impl that doesn't override the (value, ff) overload should still receive the value.
    io.setVelocity(7.0, 99.0);

    assertEquals(7.0, got[0]);
  }

  @Test
  void motionMagicPositionFeedforwardOverloadDelegatesToPlain() {
    final double[] got = {Double.NaN};
    MotorIO io =
        new MotorIO() {
          @Override
          public void setMotionMagicPosition(double positionRad) {
            got[0] = positionRad;
          }
        };

    io.setMotionMagicPosition(1.5, 42.0);

    assertEquals(1.5, got[0]);
  }
}
