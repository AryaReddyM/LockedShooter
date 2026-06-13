package frc.robot.subsystems.shooter.flywheel;

import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends StateMachine<Flywheel.State> {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private final RobotState state;

  private double tunedSetpointRPS = 100.0;
  private double rpsMultiplier = 1.0;
  private boolean ready = false;

  public Flywheel(MotorIO io, RobotState state) {
    super("Flywheel", State.UNDETERMINED, State.class);
    this.io = io;
    this.state = state;

    addOmniTransitions(State.IDLE, State.SHOOT, State.PASS, State.TRACKING, State.TUNING);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    double desiredRPS =
        switch (getState()) {
          case SHOOT -> state.getCurrentHubSetpoint().getShooterRPS() * rpsMultiplier;
          case PASS -> state.getCurrentPassSetpoint().getShooterRPS();
          case TRACKING -> FlywheelConstants.kSlowSpeed;
          case TUNING -> tunedSetpointRPS;
          default -> 0.0;
        };

    Setpoint.motionMagicVelocity(desiredRPS).apply(io);
    ready = computeReady(desiredRPS, FlywheelConstants.kFlywheelSpeedTolerance);

    Logger.recordOutput("Flywheel/DesiredRPS", desiredRPS);
    Logger.recordOutput("Flywheel/Ready", ready);
  }

  private boolean computeReady(double desiredRPS, double tolerance) {
    if (desiredRPS < 1) {
      return false;
    }
    return (desiredRPS - tolerance) < inputs.velocityRadPerSec;
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public boolean isReady() {
    return ready;
  }

  public void setMultiplier(double newMultiplier) {
    rpsMultiplier = newMultiplier;
  }

  public double getMultiplier() {
    return rpsMultiplier;
  }

  public void setTuningSetpointRPS(double rps) {
    tunedSetpointRPS = rps;
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    SHOOT,
    PASS,
    TRACKING,
    TUNING
  }
}
