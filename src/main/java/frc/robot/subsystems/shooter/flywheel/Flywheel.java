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

  /** Tuning setpoint, in flywheel surface m/s (the flywheel's native unit). */
  private double tunedSetpointSurfaceSpeed = 15.0;

  private double speedMultiplier = 1.0;
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

    double desiredSurfaceSpeed =
        switch (getState()) {
          case SHOOT -> state.getCurrentHubSetpoint().getFlywheelSurfaceSpeed() * speedMultiplier;
          case PASS -> state.getCurrentPassSetpoint().getFlywheelSurfaceSpeed();
          case TRACKING -> FlywheelConstants.kSlowSpeed;
          case TUNING -> tunedSetpointSurfaceSpeed;
          default -> 0.0;
        };

    Setpoint.motionMagicVelocity(desiredSurfaceSpeed).apply(io);
    ready = computeReady(desiredSurfaceSpeed, FlywheelConstants.kFlywheelSpeedTolerance);

    Logger.recordOutput("Flywheel/DesiredSurfaceSpeed", desiredSurfaceSpeed);
    Logger.recordOutput("Flywheel/MeasuredSurfaceSpeed", inputs.velocityRadPerSec);
    Logger.recordOutput("Flywheel/Ready", ready);
  }

  /**
   * Both sides of this comparison are flywheel surface speeds in m/s. The measurement lives in
   * {@code velocityRadPerSec} only because that is what the shared MotorIO inputs call the field;
   * the flywheel's conversion factor makes it surface speed. Being over the setpoint still counts
   * as ready, and the sign is taken out so a reversed shooter behaves the same.
   */
  private boolean computeReady(double desiredSurfaceSpeed, double tolerance) {
    if (Math.abs(desiredSurfaceSpeed) < 1e-3) {
      return false;
    }
    return Math.abs(inputs.velocityRadPerSec) > Math.abs(desiredSurfaceSpeed) - tolerance;
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public boolean isReady() {
    return ready;
  }

  /**
   * Total surface travel in meters. Divided by the wheel radius this is the wheel's angle, which
   * is all a visualiser needs to spin it.
   */
  public double getSurfaceTravelMeters() {
    return inputs.positionRad;
  }

  public void setMultiplier(double newMultiplier) {
    speedMultiplier = newMultiplier;
  }

  public double getMultiplier() {
    return speedMultiplier;
  }

  /** @param surfaceSpeed flywheel surface speed in m/s */
  public void setTuningSetpoint(double surfaceSpeed) {
    tunedSetpointSurfaceSpeed = surfaceSpeed;
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
