package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;

public final class AutoCommands {
  private AutoCommands() {}

  public static Command none() {
    return Commands.none().withName("None");
  }

  public static Command shootPreload(RobotState state) {
    return ActionCommands.shoot(state).withTimeout(5.0).withName("ShootPreload");
  }
}
