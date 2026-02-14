package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
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

    // DONT FORGET THIS
    private static final List<AutoClass> availableAutos = List.of(
            new testAuto(),
            new waypointTestAuto(),
            new depotAuto(),
            new rightHPFuel(),
            new Autos.centerHPClimb(),
            new Autos.centerHPFuel(),
            new Autos.centerLeftDepotClimb(),
            new Autos.centerRightHPClimb(),
            new Autos.leftDepotClimb(),
            new Autos.leftDepotFuel(),
            new Autos.rightFuelClimb(),
            new Autos.rightHPFuel()
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
        public List<PathPlannerPath> getAutoDisplayList() {
            List<PathPlannerPath> pathList = new ArrayList<>();

            if (waypointGeneratedPath != null) {
                pathList.add(waypointGeneratedPath);
            }

            return pathList;
        }
    }

    public static class depotAuto extends AutoClass {
        Pose2d tagPos;

        public depotAuto() {
            this.name = "Depot (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Starting to Depot",
                    "Depot to 1st Shooting",
                    "1st Shooting to 2nd Shooting"
            };

            tagPos = VisionConstants.kAprilTagLayout.getTagPose(7).get().toPose2d()
                    .plus(new Transform2d(Units.inchesToMeters(36), Units.inchesToMeters(0),
                            new Rotation2d(Units.degreesToRadians(270))));
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);
                new ActionCommands();

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        AutoBuilder.followPath(pathMap.get("Starting to Depot")),
                        new WaitCommand(2), // Temp seconds amount
                        AutoBuilder.followPath(pathMap.get("Depot to 1st Shooting")),
                        // ActionCommands.aimAndShoot(state),
                        AutoBuilder.followPath(pathMap.get("1st Shooting to 2nd Shooting")),
                        // ActionCommands.aimAndShoot(state),
                        new AutoAlignToPoseCommand(state.getDrive(), state, tagPos, 0)
                // ActionCommands.climbUp(state)
                )
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class rightHPFuel extends AutoClass {
        public rightHPFuel() {
            this.name = "Right HP Fuel (GAME)";
            this.sequentialPathStrings = new String[] { "Right to Center", "Center to Right Fuel",
                    "Right Fuel to Center", "Center to HP", "HP Pickup", "HP to Center" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Right to Center")),
                                new SequentialCommandGroup(
                                new WaitCommand(AutosConstants.rightToCenter)
                                // new InstantCommand(() -> {
                                // //state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                                // shooter is ready

                                // })
                                )),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                                new SequentialCommandGroup(
                                new WaitCommand(AutosConstants.centerToRFuel)
                                // new InstantCommand(() -> {
                                // //state.getIntake().requestTransition(State.INTAKING); // uncomment when
                                // intake is ready
                                // })
                                )),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                                new SequentialCommandGroup(
                                new WaitCommand(AutosConstants.rightToCenter)
                                // new InstantCommand(() -> {
                                // //state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                                // shooter is ready
                                // })
                                )),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to HP")),
                                new SequentialCommandGroup(
                                new WaitCommand(AutosConstants.centerToHP)
                                // new InstantCommand(() -> {
                                // //state.getIntake().requestTransition(State.INTAKING); // uncomment when
                                // intake is ready
                                // })
                                )),
                        AutoBuilder.followPath(pathMap.get("HP Pickup")),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("HP to Center")),
                                new SequentialCommandGroup(
                                new WaitCommand(AutosConstants.hpToCenter)
                                // new InstantCommand(() -> {
                                // //state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                                // shooter is ready
                                // })
                                ))).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

}
