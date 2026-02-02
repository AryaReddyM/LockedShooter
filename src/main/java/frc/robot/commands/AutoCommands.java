package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Shooter.State;
import frc.robot.subsystems.shooter.turret.Turret;

public class AutoCommands {

    public static PathPlannerAuto getRawAutoWithCommand(Command command) {
        return new PathPlannerAuto(command.getName());
    }

    public static Command test(RobotState state) {
        try {
            PathPlannerPath pathTest = PathPlannerPath.fromPathFile("Applesauce");
            return new ParallelCommandGroup(
              AutoBuilder.followPath(pathTest),
              new SequentialCommandGroup(
                new WaitCommand(1),
                new InstantCommand(() -> {
                    state.getShooter().requestTransition(State.SHOOTING);
                })
              )  
            );
        } catch(Exception e) {
            return new PrintCommand("Failed to generate command");
        }
    }

    public static Command getAuto(String autoName) {
        return new PathPlannerAuto(autoName).withName(autoName);
    }

    public static Command testingAuto() {
        return getAuto("Auto Testing");
    }
}
