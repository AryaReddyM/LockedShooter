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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.util.CustomAutoBuilder;
import frc.robot.util.DynamicPathGenerator;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

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

        public void setRobotPoseToStartingPath(PathPlannerPath path, RobotState state) {
            Optional<Alliance> ourAlliance = DriverStation.getAlliance();

            if (ourAlliance.isPresent()) {
                if (ourAlliance.get().equals(Alliance.Red)) {
                    path.flipPath();
                }

                if (path.getStartingHolonomicPose().isEmpty()) {
                    Elastic.sendNotification(new Notification().withTitle("Path Error").withDescription("Unable to set pose").withLevel(NotificationLevel.ERROR));
                }

                state.getDrive().setPose(path.getStartingHolonomicPose().get());
            } else {
                Elastic.sendNotification(new Notification().withTitle("Alliance Error").withDescription("Unable to set pose").withLevel(NotificationLevel.ERROR));
            }
        }
    }

    // DONT FORGET THIS
    private static final List<AutoClass> availableAutos = List.of(
            new testAuto(),
            new waypointTestAuto(),
            new depotAuto()
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
            this.sequentialPathStrings = new String[] { "TESTONE" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);

                return new ParallelCommandGroup(
                        new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        AutoBuilder.followPath(pathMap.get("TESTONE")),
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

    public static class depotAuto extends AutoClass {
        public depotAuto() {
            this.name = "Depot (GAME)";
            this.sequentialPathStrings = new String[] {
                "Starting to Depot", 
                "Depot to 1st Shooting",
                "1st Shooting to 2nd Shooting"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);
                new ActionCommands();
                
                return new SequentialCommandGroup(
                    AutoBuilder.followPath(pathMap.get("Starting to Depot")),
                    new WaitCommand(2), // Temp seconds amount
                    AutoBuilder.followPath(pathMap.get("Depot to 1st Shooting")),
                    //ActionCommands.aimAndShoot(state),
                    AutoBuilder.followPath(pathMap.get("1st Shooting to 2nd Shooting")),
                    //ActionCommands.aimAndShoot(state),
                    new AutoAlignToPoseCommand(state.getDrive(), state, state.getAprilTagPose(7), 0)
                    //ActionCommands.climbUp(state)
                )
                .withName(name);
            }
            catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }
}