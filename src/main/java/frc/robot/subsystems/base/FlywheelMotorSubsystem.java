package frc.robot.subsystems.base;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FlywheelMotorSubsystem extends MotorSubsystem {
  private final double toleranceRadPerSec;

  public FlywheelMotorSubsystem(MotorIO io, String name, double toleranceRadPerSec) {
    super(io, name);
    this.toleranceRadPerSec = toleranceRadPerSec;
  }

  @AutoLogOutput(key = "{name}/AtSpeed")
  public boolean atSpeed() {
    return MathUtil.isNear(getSetpoint().getValue(), getVelocityRadPerSec(), toleranceRadPerSec);
  }

  public Command spinAndWait(Setpoint setpoint) {
    return runOnce(() -> applySetpoint(setpoint)).andThen(Commands.waitUntil(this::atSpeed));
  }

  public Command spinUpAndWait(Setpoint setpoint) {
    return spinAndWait(setpoint);
  }
}
