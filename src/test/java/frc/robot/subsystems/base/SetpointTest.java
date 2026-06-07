package frc.robot.subsystems.base;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SetpointTest {
  private static class FakeMotorIO implements MotorIO {
    String lastCall = "";
    double value = Double.NaN;
    double feedforward = Double.NaN;
    boolean stopped = false;

    @Override
    public void setVoltage(double volts) {
      lastCall = "voltage";
      value = volts;
    }

    @Override
    public void setPosition(double positionRad, double feedforwardVolts) {
      lastCall = "position";
      value = positionRad;
      feedforward = feedforwardVolts;
    }

    @Override
    public void setMotionMagicPosition(double positionRad, double feedforwardVolts) {
      lastCall = "motionMagicPosition";
      value = positionRad;
      feedforward = feedforwardVolts;
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double feedforwardVolts) {
      lastCall = "velocity";
      value = velocityRadPerSec;
      feedforward = feedforwardVolts;
    }

    @Override
    public void setMotionMagicVelocity(double velocityRadPerSec, double feedforwardVolts) {
      lastCall = "motionMagicVelocity";
      value = velocityRadPerSec;
      feedforward = feedforwardVolts;
    }

    @Override
    public void stop() {
      lastCall = "stop";
      stopped = true;
    }
  }

  @Test
  void motionMagicAliasAppliesProfiledPosition() {
    FakeMotorIO io = new FakeMotorIO();

    Setpoint.motionMagic(1.25).applyWithFeedforward(io, 0.4);

    assertEquals("motionMagicPosition", io.lastCall);
    assertEquals(1.25, io.value);
    assertEquals(0.4, io.feedforward);
  }

  @Test
  void velocityAppliesClosedLoopVelocity() {
    FakeMotorIO io = new FakeMotorIO();

    Setpoint.velocity(8.0).applyWithFeedforward(io, 1.2);

    assertEquals("velocity", io.lastCall);
    assertEquals(8.0, io.value);
    assertEquals(1.2, io.feedforward);
  }

  @Test
  void idleStopsMotor() {
    FakeMotorIO io = new FakeMotorIO();

    Setpoint.idle().apply(io);

    assertEquals("stop", io.lastCall);
    assertEquals(true, io.stopped);
  }
}

