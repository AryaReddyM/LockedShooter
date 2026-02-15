package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.autos.AutosConstants;
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

        public List<PathPlannerPath> getAutoDisplayList(RobotState state) {
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
                if (path.getStartingHolonomicPose().isEmpty()) {
                    Elastic.sendNotification(new Notification().withTitle("Path Error")
                            .withDescription("Unable to set pose").withLevel(NotificationLevel.ERROR));
                }

                Pose2d startPos = path.getStartingHolonomicPose().get();
                if (ourAlliance.get().equals(Alliance.Red)) {
                    startPos = RobotState.flipPoseForRed(startPos);
                }

                state.getDrive().setPose(startPos);
            } else {
                Elastic.sendNotification(new Notification().withTitle("Alliance Error")
                        .withDescription("Unable to set pose").withLevel(NotificationLevel.ERROR));
            }
        }
    }

    // // DONT FORGET THIS
    // private static final List<AutoClass> availableAutos = List.of(
    //         new testAuto(),
    //         new waypointTestAuto(),
    //         new pathfindingTemplate()
    //         // new depotAuto(),
    //         // new rightHPFuel(),
    //         // new Autos.centerHPClimb(),
    //         // new Autos.centerHPFuel(),
    //         // new Autos.centerLeftDepotClimb(),
    //         // new Autos.centerRightHPClimb(),
    //         // new Autos.leftDepotClimb(),
    //         // new Autos.leftDepotFuel(),
    //         // new Autos.rightFuelClimb(),
    //         // new Autos.rightHPFuel(),
    //         // new outpostAuto()
    // );

    private static final List<AutoClass> availableAutos = initializeAutos();

private static List<AutoClass> initializeAutos() {
    List<AutoClass> autos = new ArrayList<>();

    autos.add(new testAuto());
    autos.add(new waypointTestAuto());
    autos.add(new pathfindingTemplate());

    Class<?>[] innerClasses = Autos.class.getDeclaredClasses();
    for (Class<?> clazz : innerClasses) {
        if (AutoClass.class.isAssignableFrom(clazz)) {
            try {
                AutoClass instance = (AutoClass) clazz.getDeclaredConstructor().newInstance();
                if (autos.stream().noneMatch(a -> a.getClass().equals(clazz))) {
                    autos.add(instance);
                }
            } catch (Exception e) {
                System.out.println("Skipping " + clazz.getSimpleName() + ": " + e.getMessage());
            }
        }
    }

    return autos;
}

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

    public static Map<String, PathPlannerPath> getMapPath(String[] sequentialPathStrings) throws Exception {
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
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        AutoBuilder.followPath(pathMap.get("TESTONE")),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    // state.getShooter().requestTransition(State.SHOOTING);
                                })))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
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
                            new InstantCommand()))
                    .withName(name);
        }

        @Override
        public List<PathPlannerPath> getAutoDisplayList(RobotState state) {
            List<PathPlannerPath> pathList = new ArrayList<>();

            if (waypointGeneratedPath != null) {
                pathList.add(waypointGeneratedPath);
            }

            return pathList;
        }
    }

    public static class pathfindingTemplate extends AutoClass {
        
        List<Command> pathfindAutos;
        List<Pose2d> goalPoses;

        PathConstraints constraints = DriveConstants.pathConstraint;
        // Pose2d goalPose = new Pose2d(5, 5, Rotation2d.fromDegrees(180));

        public pathfindingTemplate() {
            this.name = "Pathfinding (GAME)";
            this.sequentialPathStrings = new String[] {};


            pathfindAutos = new ArrayList<>();
            goalPoses = new ArrayList<>();

            Pose2d goalPose = new Pose2d(6, 5, Rotation2d.fromDegrees(180));
            pathfindAutos.add(DynamicPathGenerator.pathfindAuto(goalPose));
            goalPoses.add(goalPose);

            Pose2d depotGoalPose = new Pose2d(
                    VisionConstants.Outpost.centerPoint.getX(),
                    VisionConstants.Outpost.centerPoint.getY(),
                    Rotation2d.fromDegrees(0) // intake looking at the place?
                );
            pathfindAutos.add(DynamicPathGenerator.pathfindAuto(depotGoalPose));
            goalPoses.add(depotGoalPose);
        }

        @Override
        public Command getCommand(RobotState state) {
            return new SequentialCommandGroup(
            pathfindAutos.get(0),
            new InstantCommand(() -> {
                state.getDrive().setPose(goalPoses.get(0));
            }),
            pathfindAutos.get(1)
            ).withName(name);
        }

        @Override
        public List<PathPlannerPath> getAutoDisplayList(RobotState state) {
            List<PathPlannerPath> pathList = new ArrayList<>();

            pathList.add(DynamicPathGenerator.getPathFromWaypoints(PathPlannerPath.waypointsFromPoses(
                state.getLatestFieldToRobot().getValue(),
                goalPoses.get(0)
            ), Optional.of(this.constraints), new IdealStartingState(0, state.getLatestFieldToRobot().getValue().getRotation()),new GoalEndState(0, goalPoses.get(0).getRotation())));

            pathList.add(DynamicPathGenerator.getPathFromWaypoints(PathPlannerPath.waypointsFromPoses(
                goalPoses.get(0),
                goalPoses.get(1)
            ), Optional.of(this.constraints), new IdealStartingState(0, goalPoses.get(0).getRotation()),new GoalEndState(0, goalPoses.get(1).getRotation())));

            return pathList;
        }
    }
}
