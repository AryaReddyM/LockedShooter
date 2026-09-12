// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;

public class RobotContainer {
  private final RobotState robotState = new RobotState();

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  public RobotState getRobotState() {
    return robotState;
  }

  private void configureBindings() {
    robotState
        .getDrive()
        .setDefaultCommand(
            DriveCommands.joystickDrive(
                robotState.getDrive(),
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

    driverController
        .a()
        .whileTrue(ActionCommands.intake(robotState))
        .onFalse(ActionCommands.idle(robotState));

    driverController
        .b()
        .whileTrue(ActionCommands.shoot(robotState))
        .onFalse(ActionCommands.idle(robotState));

    driverController
        .x()
        .whileTrue(ActionCommands.pass(robotState))
        .onFalse(ActionCommands.idle(robotState));

    driverController.y().onTrue(ActionCommands.climb(robotState));
  }

  public Command getAutonomousCommand() {
    return AutoCommands.shootPreload(robotState);
  }
}
