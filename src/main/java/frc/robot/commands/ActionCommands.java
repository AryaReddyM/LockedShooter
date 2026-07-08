package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;

public final class ActionCommands {
  private ActionCommands() {}

  public static Command intake(RobotState state) {
    Superstructure ss = state.getSuperstructure();
    return ss.transitionCommand(Superstructure.State.INTAKING)
        .andThen(ss.transitionCommand(Superstructure.State.IDLE))
        .withName("Intake");
  }

  public static Command aim(RobotState state) {
    return state
        .getSuperstructure()
        .transitionCommand(Superstructure.State.HUB_TRACKING)
        .withName("Aim");
  }

  public static Command shoot(RobotState state) {
    Superstructure ss = state.getSuperstructure();
    return ss.transitionCommand(Superstructure.State.HUB_TRACKING)
        .andThen(waitUntilReady(state))
        .andThen(ss.transitionCommand(Superstructure.State.SHOOTING))
        .withName("Shoot");
  }

  public static Command pass(RobotState state) {
    return state
        .getSuperstructure()
        .transitionCommand(Superstructure.State.PASSING)
        .withName("Pass");
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

  private static Command waitUntilReady(RobotState state) {
    return state
        .getShooter()
        .waitForState(frc.robot.subsystems.shooter.Shooter.State.HUB_TRACKING);
  }
}
