package frc.robot.subsystems.hopper;

import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Hopper extends StateMachine<Hopper.State> {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  public static final Setpoint SHOOT_SPEED =
      Setpoint.motionMagicVelocity(HopperConstants.kHopperShootSpeed);
  public static final Setpoint OUTTAKE_SPEED =
      Setpoint.motionMagicVelocity(HopperConstants.kHopperOuttakeSpeed);

  public Hopper(MotorIO io) {
    super("Hopper", State.UNDETERMINED, State.class);
    this.io = io;

    addOmniTransitions(State.IDLE, State.SHOOT, State.OUTTAKE);

    registerStateCommand(State.IDLE, () -> io.stop());
    registerStateCommand(State.SHOOT, () -> SHOOT_SPEED.apply(io));
    registerStateCommand(State.OUTTAKE, () -> OUTTAKE_SPEED.apply(io));
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    SHOOT,
    OUTTAKE
  }
}
