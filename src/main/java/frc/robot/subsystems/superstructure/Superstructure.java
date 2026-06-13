package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.state.StateMachine;

public class Superstructure extends StateMachine<Superstructure.State> {
  private final Shooter shooter;
  private final Intake intake;
  private final Hopper hopper;
  private final Kicker kicker;
  private final Climb climb;

  public Superstructure(
      Shooter shooter, Intake intake, Hopper hopper, Kicker kicker, Climb climb) {
    super("Superstructure", State.UNDETERMINED, State.class);
    this.shooter = shooter;
    this.intake = intake;
    this.hopper = hopper;
    this.kicker = kicker;
    this.climb = climb;

    addOmniTransitions(
        State.IDLE,
        State.TRAVERSING,
        State.INTAKING,
        State.HUB_TRACKING,
        State.SHOOTING,
        State.PASSING,
        State.CLIMBING);

    registerStateCommand(State.IDLE, () -> applyIdle());
    registerStateCommand(State.TRAVERSING, () -> applyIdle());

    registerStateCommand(
        State.INTAKING,
        cascade(
            Shooter.State.IDLE,
            Intake.State.INTAKE,
            Hopper.State.IDLE,
            Kicker.State.IDLE,
            Climb.State.STOW));

    registerStateCommand(
        State.HUB_TRACKING,
        cascade(
            Shooter.State.HUB_TRACKING,
            Intake.State.STOW,
            Hopper.State.IDLE,
            Kicker.State.IDLE,
            Climb.State.STOW));

    registerStateCommand(
        State.SHOOTING,
        cascade(
            Shooter.State.SHOOTING,
            Intake.State.STOW,
            Hopper.State.SHOOT,
            Kicker.State.SHOOT,
            Climb.State.STOW));

    registerStateCommand(
        State.PASSING,
        cascade(
            Shooter.State.PASSING,
            Intake.State.STOW,
            Hopper.State.SHOOT,
            Kicker.State.SHOOT,
            Climb.State.STOW));

    registerStateCommand(
        State.CLIMBING,
        cascade(
            Shooter.State.IDLE,
            Intake.State.CLIMB_TOW,
            Hopper.State.IDLE,
            Kicker.State.IDLE,
            Climb.State.UP));
  }

  private void applyIdle() {
    shooter.requestTransition(Shooter.State.IDLE);
    intake.requestTransition(Intake.State.STOW);
    hopper.requestTransition(Hopper.State.IDLE);
    kicker.requestTransition(Kicker.State.IDLE);
    climb.requestTransition(Climb.State.STOW);
  }

  private InstantCommand cascade(
      Shooter.State shooterState,
      Intake.State intakeState,
      Hopper.State hopperState,
      Kicker.State kickerState,
      Climb.State climbState) {
    return new InstantCommand(
        () -> {
          shooter.requestTransition(shooterState);
          intake.requestTransition(intakeState);
          hopper.requestTransition(hopperState);
          kicker.requestTransition(kickerState);
          climb.requestTransition(climbState);
        });
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    TRAVERSING,
    INTAKING,
    HUB_TRACKING,
    SHOOTING,
    PASSING,
    CLIMBING
  }
}
