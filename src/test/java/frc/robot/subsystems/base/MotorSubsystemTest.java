package frc.robot.subsystems.base;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class MotorSubsystemTest {
  private static class FakeMotorIO implements MotorIO {
    double velocity = Double.NaN;

    @Override
    public void setVelocity(double velocityRadPerSec) {
      velocity = velocityRadPerSec;
    }
  }

  @Test
  void applySetpointStoresAndAppliesCurrentSetpoint() {
    FakeMotorIO io = new FakeMotorIO();
    MotorSubsystem subsystem = new MotorSubsystem(io, "TestMotor");
    Setpoint setpoint = Setpoint.velocity(5.0);

    subsystem.applySetpoint(setpoint);

    assertEquals(setpoint, subsystem.getSetpoint());
    assertEquals(5.0, io.velocity);
  }
}
