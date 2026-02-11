package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.util.CustomAutoBuilder;
import frc.robot.util.DynamicPathGenerator;

public class AutoCommands {

    public static class AutoClass {
        public String[] sequentialPathStrings;
        public String name;

        public Command getCommand(RobotState state) {
            return new PrintCommand("Unfilled");
        }

        public List<PathPlannerPath> getAutoDisplayList() {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);

                return new ArrayList<>(pathMap.values());
            } catch (Exception e) {
                return new ArrayList<>();
            }
        }
    }

    // DONT FORGET THIS
    private static final List<AutoClass> availableAutos = List.of(
            new testAuto(),
            new waypointTestAuto(),
            new centerToFuelToCenter(),
            new rightFuelClimb(),
            new leftDepotClimb(),
            new centerHPClimb(),
            new centerRightHPClimb(),
            new centerLeftDepotClimb(),
            new leftDepotFuel(),
            new centerHPFuel(),
            new rightHPFuel()
    );

    public static Optional<AutoClass> getAutoByName(RobotState state, String name) {
        if (name == "CUSTOM AUTO (GAME)") {
            return Optional.of(state.getCustomAutoBuilder());
        }
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
        public testAuto() {
            this.name = "Apple (GAME)";
            this.sequentialPathStrings = new String[] { "Center to Depot" };
        }

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
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }

    public static class waypointTestAuto extends AutoClass {
        PathPlannerPath waypointGeneratedPath;

        public waypointTestAuto() {
            this.name = "WAYPOINT (GAME)";
            this.sequentialPathStrings = new String[] {};
            waypointGeneratedPath = ActionCommands.waypointTestPath();
        }

        @Override
        public Command getCommand(RobotState state) {
            return new ParallelCommandGroup(
                    AutoBuilder.followPath(waypointGeneratedPath),
                    new SequentialCommandGroup(
                            new WaitCommand(1),
                            new InstantCommand())).withName(name);
        }

        @Override
        public List<PathPlannerPath> getAutoDisplayList() {
            List<PathPlannerPath> pathList = new ArrayList<>();

            if (waypointGeneratedPath != null) {
                pathList.add(waypointGeneratedPath);
            }

            return pathList;
        }
    }
}
