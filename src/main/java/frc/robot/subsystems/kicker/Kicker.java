package frc.robot.subsystems.kicker;

import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Kicker extends StateMachine<Kicker.State> {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  public static final Setpoint SHOOT_SPEED =
      Setpoint.motionMagicVelocity(KickerConstants.kKickerShootSpeed);
  public static final Setpoint OUTTAKE_SPEED =
      Setpoint.motionMagicVelocity(KickerConstants.kKickerOutakeSpeed);

  public Kicker(MotorIO io) {
    super("Kicker", State.UNDETERMINED, State.class);
    this.io = io;

    addOmniTransitions(State.IDLE, State.SHOOT, State.OUTTAKE);

    registerStateCommand(State.IDLE, () -> io.stop());
    registerStateCommand(State.SHOOT, () -> SHOOT_SPEED.apply(io));
    registerStateCommand(State.OUTTAKE, () -> OUTTAKE_SPEED.apply(io));
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public boolean atSpeed(double toleranceRadPerSec) {
    return Math.abs(inputs.velocityRadPerSec - getSetpoint().getValue()) < toleranceRadPerSec;
  }

  private Setpoint getSetpoint() {
    return switch (getState()) {
      case SHOOT -> SHOOT_SPEED;
      case OUTTAKE -> OUTTAKE_SPEED;
      default -> Setpoint.idle();
    };
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    SHOOT,
    OUTTAKE
  }
}
