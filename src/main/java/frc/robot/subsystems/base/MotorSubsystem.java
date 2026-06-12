package frc.robot.subsystems.base;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
  protected final MotorIO io;
  protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  protected final String name;

  private Setpoint currSetpoint = Setpoint.idle();

  public MotorSubsystem(MotorIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    Logger.processInputs(name, inputs);
  }

  public void applySetpoint(Setpoint setpoint) {
    currSetpoint = setpoint;
    setpoint.apply(io);
    Logger.recordOutput(name + "/Setpoint", setpoint.getName());
    Logger.recordOutput(name + "/SetpointValue", setpoint.getValue());
    Logger.recordOutput(name + "/Feedforward", 0.0);
  }
  
  public void applySetpoint(Setpoint setpoint, double feedforwardVolts) {
    currSetpoint = setpoint;
    currSetpoint.applyWithFeedforward(io, feedforwardVolts);
    Logger.recordOutput(name + "/Setpoint", setpoint.getName());
    Logger.recordOutput(name + "/SetpointValue", setpoint.getValue());
    Logger.recordOutput(name + "/Feedforward", feedforwardVolts);
  }

  public Setpoint getSetpoint() {
    return currSetpoint;
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public void stop() {
    applySetpoint(Setpoint.idle());
  }

  public Command setSetpoint(Setpoint setpoint) {
    return runOnce(() -> applySetpoint(setpoint));
  }

  public Command holdSetpoint(Setpoint setpoint) {
    return run(() -> applySetpoint(setpoint));
  }

  public Command holdSetpoint(Setpoint setpoint, DoubleSupplier feedforwardVolts) {
    return run(() -> applySetpoint(setpoint, feedforwardVolts.getAsDouble()));
  }
}
