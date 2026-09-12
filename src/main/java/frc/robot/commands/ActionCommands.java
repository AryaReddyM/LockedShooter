package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;

public final class ActionCommands {
  private ActionCommands() {}

  /**
   * Holds the superstructure in a state until the command is interrupted.
   *
   * <p>{@link Superstructure#transitionCommand} finishes the moment the state is reached, so on its
   * own a {@code whileTrue} binding would enter the state and immediately end, and anything chained
   * after it would fire straight away rather than when the button is released.
   */
  private static Command hold(RobotState state, Superstructure.State target) {
    return state.getSuperstructure().transitionCommand(target).andThen(Commands.idle());
  }

  public static Command intake(RobotState state) {
    return hold(state, Superstructure.State.INTAKING).withName("Intake");
  }

  public static Command aim(RobotState state) {
    return hold(state, Superstructure.State.HUB_TRACKING).withName("Aim");
  }

  /** Tracks the hub, waits until the shot is actually takeable, then feeds. */
  public static Command shoot(RobotState state) {
    Superstructure ss = state.getSuperstructure();
    return ss.transitionCommand(Superstructure.State.HUB_TRACKING)
        .andThen(waitUntilReady(state))
        .andThen(hold(state, Superstructure.State.SHOOTING))
        .withName("Shoot");
  }

  public static Command pass(RobotState state) {
    return hold(state, Superstructure.State.PASSING).withName("Pass");
  }

  public static Command climb(RobotState state) {
    return state
        .getSuperstructure()
        .transitionCommand(Superstructure.State.CLIMBING)
        .withName("Climb");
  }

  public static Command idle(RobotState state) {
    return state
        .getSuperstructure()
        .transitionCommand(Superstructure.State.IDLE)
        .withName("Idle");
  }

  /**
   * Waits for the turret, hood and flywheel to all be on target with a shot that is in range. This
   * used to wait on the shooter reaching HUB_TRACKING, which the previous step had already done, so
   * it returned immediately and the kicker fed fuel into a flywheel that was still spinning up.
   */
  private static Command waitUntilReady(RobotState state) {
    return Commands.waitUntil(
        () ->
            state.getCurrentHubSetpoint().isAchievable() && state.getShooter().readyToShoot());
  }
}
