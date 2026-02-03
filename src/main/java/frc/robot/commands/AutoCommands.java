package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;

public class AutoCommands {

    public static class AutoClass {
        String[] sequentialPathStrings;
        String name;

        public Command getCommand(RobotState state) {
            return new PrintCommand("Unfilled");
        }

        public List<PathPlannerPath> getAutoDisplayList() {
            return new ArrayList<>();
        }
    }

    private static final List<AutoClass> availableAutos = List.of(
        new testAuto()
    );

    public static Optional<AutoClass> getAutoByName(String name) {
        for (AutoClass auto : availableAutos) {
            if (auto.name.equals(name)) {
                return Optional.of(auto);
            }
        }
        
        return Optional.empty(); 
    }

    private static Map<String, PathPlannerPath> getMapPath(String[] sequentialPathStrings) throws Exception {
        Map<String, PathPlannerPath> pathMap = new HashMap<>();

        for (String pathName : sequentialPathStrings) {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            pathMap.put(pathName, path);
        }

        return pathMap;
    }

    public static class testAuto extends AutoClass {
        String[] sequentialPathStrings = { "Center to Depot" };
        String name = "Apple";

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);

                return new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Depot")),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    // state.getShooter().requestTransition(State.SHOOTING);
                                })))
                        .withName(name + " GAME");
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }

        @Override
        public List<PathPlannerPath> getAutoDisplayList() {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);
                return new ArrayList<>(pathMap.values());
            } catch (Exception e) {
                return new ArrayList<>();
            }
        }
    }
}
