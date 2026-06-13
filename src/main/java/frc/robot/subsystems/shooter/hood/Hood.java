package frc.robot.subsystems.shooter.hood;

import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Hood extends StateMachine<Hood.State> {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private final RobotState state;

  private double tunedSetpoint = HoodConstants.kHoodMinLimit;
  private double desiredPos = 0.0;

  public Hood(MotorIO io, RobotState state) {
    super("Hood", State.UNDETERMINED, State.class);
    this.io = io;
    this.state = state;

    addOmniTransitions(State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.TUNING);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    switch (getState()) {
      case HUB_TRACKING ->
          setPos(state.getCurrentHubSetpoint().getHoodRadians(), state.getCurrentHubSetpoint().getHoodFF());
      case PASS_TRACKING ->
          setPos(state.getCurrentPassSetpoint().getHoodRadians(), state.getCurrentPassSetpoint().getHoodFF());
      case TUNING -> setPos(tunedSetpoint, 0.0);
      default -> io.stop();
    }

    Logger.recordOutput("Hood/Desired", desiredPos);
  }

  private void setPos(double positionRad, double feedforwardVolts) {
    desiredPos = positionRad;
    Setpoint.motionMagicPosition(positionRad).applyWithFeedforward(io, feedforwardVolts);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public double getHoodPosition() {
    return desiredPos;
  }

  public boolean atSetpoint(double toleranceRad) {
    return Math.abs(inputs.positionRad - desiredPos) < toleranceRad;
  }

  public void setTuningSetpoint(double radians) {
    tunedSetpoint = radians;
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    PASS_TRACKING,
    HUB_TRACKING,
    TUNING
  }
}
