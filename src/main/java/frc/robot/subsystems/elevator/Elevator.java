package frc.robot.subsystems.elevator;

import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachine<Elevator.State> {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  public static final Setpoint STOW = Setpoint.motionMagicPosition(ElevatorConstants.kStowHeight);
  public static final Setpoint LOW = Setpoint.motionMagicPosition(ElevatorConstants.kLowHeight);
  public static final Setpoint HIGH = Setpoint.motionMagicPosition(ElevatorConstants.kHighHeight);

  public Elevator(MotorIO io) {
    super("Elevator", State.UNDETERMINED, State.class);
    this.io = io;

    addOmniTransitions(State.IDLE, State.STOW, State.LOW, State.HIGH);

    registerStateCommand(State.IDLE, () -> io.stop());
    registerStateCommand(
        State.STOW, () -> STOW.applyWithFeedforward(io, ElevatorConstants.kElevatorG));
    registerStateCommand(
        State.LOW, () -> LOW.applyWithFeedforward(io, ElevatorConstants.kElevatorG));
    registerStateCommand(
        State.HIGH, () -> HIGH.applyWithFeedforward(io, ElevatorConstants.kElevatorG));
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  @Override
  protected void determineSelf() {
    setState(State.STOW);
  }

  public boolean atSetpoint() {
    return Math.abs(inputs.positionRad - targetHeight()) < ElevatorConstants.kEpsilon;
  }

  private double targetHeight() {
    return switch (getState()) {
      case STOW -> ElevatorConstants.kStowHeight;
      case LOW -> ElevatorConstants.kLowHeight;
      case HIGH -> ElevatorConstants.kHighHeight;
      default -> inputs.positionRad;
    };
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    STOW,
    LOW,
    HIGH
  }
}
