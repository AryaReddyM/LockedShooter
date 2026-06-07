package frc.robot.subsystems.base;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ServoMotorSubsystem extends MotorSubsystem {
  private final double epsilonRad;

  public ServoMotorSubsystem(MotorIO io, String name, double epsilonRad) {
    super(io, name);
    this.epsilonRad = epsilonRad;
  }

  public boolean nearPosition(double targetRad) {
    return MathUtil.isNear(targetRad, getPositionRad(), epsilonRad);
  }

  @AutoLogOutput(key = "{name}/AtSetpoint")
  public boolean atSetpoint() {
    return nearPosition(getSetpoint().getValue());
  }

  public void setEncoderPosition(double positionRad) {
    io.setEncoderPosition(positionRad);
  }

  public Command setpointAndWait(Setpoint setpoint) {
    return runOnce(() -> applySetpoint(setpoint)).andThen(Commands.waitUntil(this::atSetpoint));
  }
}
