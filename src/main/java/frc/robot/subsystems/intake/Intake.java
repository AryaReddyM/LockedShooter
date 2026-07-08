package frc.robot.subsystems.intake;

import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachine<Intake.State> {
  private final MotorIO extensionIo;
  private final MotorIOInputsAutoLogged extensionInputs = new MotorIOInputsAutoLogged();
  private final MotorIO rollersIo;
  private final MotorIOInputsAutoLogged rollerInputs = new MotorIOInputsAutoLogged();

  public static final Setpoint STOW_POS =
      Setpoint.motionMagicPosition(IntakeConstants.kExtensionStowSetpoint);
  public static final Setpoint INTAKE_POS =
      Setpoint.motionMagicPosition(IntakeConstants.kExtensionIntakeSetpoint);
  public static final Setpoint OUTTAKE_POS =
      Setpoint.motionMagicPosition(IntakeConstants.kExtensionOuttakeSetpoint);
  public static final Setpoint CLIMB_TOW_POS =
      Setpoint.motionMagicPosition(IntakeConstants.kExtensionClimbTowSetpoint);
  public static final Setpoint SHAKE_POS =
      Setpoint.motionMagicPosition(IntakeConstants.kExtensionShakeSetpoint);

  public Intake(MotorIO extensionIo, MotorIO rollersIo) {
    super("Intake", State.UNDETERMINED, State.class);
    this.extensionIo = extensionIo;
    this.rollersIo = rollersIo;

    addOmniTransitions(
        State.STOP,
        State.STOW,
        State.IDLE,
        State.INTAKE,
        State.OUTTAKE,
        State.CLIMB_TOW,
        State.SHAKE);

    registerStateCommand(State.STOP, this::stop);
    registerStateCommand(State.STOW, () -> apply(STOW_POS, 0.0));
    registerStateCommand(State.IDLE, () -> apply(INTAKE_POS, 0.0));
    registerStateCommand(
        State.INTAKE, () -> apply(INTAKE_POS, IntakeConstants.kRollerIntakeSpeed));
    registerStateCommand(
        State.OUTTAKE, () -> apply(OUTTAKE_POS, IntakeConstants.kRollerOuttakeSpeed));
    registerStateCommand(State.CLIMB_TOW, () -> apply(CLIMB_TOW_POS, 0.0));
    registerStateCommand(
        State.SHAKE, () -> apply(SHAKE_POS, IntakeConstants.kRollerIntakeSpeed));
  }

  private void apply(Setpoint extension, double rollerVelocity) {
    extension.apply(extensionIo);
    rollersIo.setVelocity(rollerVelocity);
  }

  private void stop() {
    extensionIo.stop();
    rollersIo.setVelocity(0.0);
  }

  @Override
  protected void update() {
    extensionIo.updateInputs(extensionInputs);
    rollersIo.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Extension", extensionInputs);
    Logger.processInputs("Intake/Rollers", rollerInputs);
  }

  @Override
  protected void determineSelf() {
    setState(State.STOW);
  }

  public enum State {
    UNDETERMINED,
    STOP,
    STOW,
    IDLE,
    INTAKE,
    OUTTAKE,
    CLIMB_TOW,
    SHAKE
  }
}
