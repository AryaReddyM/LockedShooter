package frc.robot.subsystems.base;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

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
