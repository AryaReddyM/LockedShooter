package frc.robot.commands.autos;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoAlignToPoseCommand;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoCommands.AutoClass;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.State;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.vision.VisionConstants;

public class Autos {

    //Aarush's Autos
    public static class depotSideDepotMidHalfSweep extends AutoClass {
        public depotSideDepotMidHalfSweep() {
            this.name = "Depot Side Depot Mid Half Sweep (GAME)";
            this.sequentialPathStrings = new String[] { 
                    "Depot Side to Depot", 
                    "Depot Intaking", 
                    "Depot to Mid Under Trench", 
                    "Mid Depot Side Half Sweep", 
                    "Mid Depot Half to Home Over Bump"
                };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Depot Side to Depot")),
                        new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.IDLE); 
                            }),
                        new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.HUB_TRACKING); 
                            })                        
                    ),
                    // MAYBE SHOOT DEPENDING ON HOW LONG IT TAKES TO SPIN UP IF TOO LONG SHOOT AFTER INTAKING

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Depot Intaking")),
                        new InstantCommand(() -> {
                            state.getIntake().requestTransition(Intake.State.INTAKE); 
                        })  
                    ),

                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            state.getIntake().requestTransition(Intake.State.IDLE); 
                        }),  
                        //need to test and figure out timings 
                        new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); 
                            }),
                            new WaitCommand(2), // need to figure out 
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.HUB_TRACKING); 
                            })
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(1),//need to figure out
                            AutoBuilder.followPath(pathMap.get("Depot to Mid Under Trench"))
                        )
                    ),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Mid Depot Side Half Sweep")),
                        new InstantCommand(() -> {
                            state.getIntake().requestTransition(Intake.State.INTAKE); 
                        })
                    ),
                    
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Mid Depot Half to Home Over Bump")),
                        new InstantCommand(() -> {
                            state.getIntake().requestTransition(Intake.State.STOW); 
                        })
                    ),

                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            state.getIntake().requestTransition(Intake.State.IDLE); 
                        }),
                        new InstantCommand(() -> {
                            state.getShooter().requestTransition(Shooter.State.SHOOTING); 
                        }),
                        new WaitCommand(5)//need to figure out
                    ),
                    
                    new InstantCommand(() -> {
                            state.getShooter().requestTransition(Shooter.State.HUB_TRACKING); 
                        })

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }



    //Other Autos
    public static class leftDepotClimb extends AutoClass {
        public leftDepotClimb() {
            this.name = "Left Depot Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Left to Center", "Center to Depot", "Depot to Center", "Center to Left Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Left to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.leftToCenter),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Depot")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToDepot),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Depot to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.depotToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Left Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToClimb),
                            new InstantCommand(() -> {
                                state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when climb is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }
    

    public static class rightFuelClimb extends AutoClass {
        public rightFuelClimb() {
            this.name = "Right Fuel Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Right to Center", "Center to Right Fuel", "Right Fuel to Center", "Center to Right Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rightToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready//state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToRFuel),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rFuelToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToClimb),
                            new InstantCommand(() -> {
                                state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when climb is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerHPClimb extends AutoClass {
        public centerHPClimb() {
            this.name = "Center HP Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Center to HP", "HP Pickup", "HP to Center", "Center to Right Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    AutoBuilder.followPath(pathMap.get("Center to HP")),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("HP Pickup")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToClimb),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("HP to Center")),
                        new SequentialCommandGroup(
                            // new WaitCommand(AutosConstants.hpToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToClimb),
                            new InstantCommand(() -> {
                                state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when climb is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerRightHPClimb extends AutoClass {
        public centerRightHPClimb() {
            this.name = "Center Right HP Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Center to Right Fuel", "Right Fuel to Center", "Center to HP", "HP Pickup", "HP to Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    // new ParallelCommandGroup(
                    //     state.getShooter().requestTransition(Shooter.State.SHOOTING), // uncomment when shooter is ready
                    //     new WaitCommand(AutosConstants.shootingPause)
                    // ),
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToRFuel),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rFuelToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to HP")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToHP),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    AutoBuilder.followPath(pathMap.get("HP Pickup")),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("HP to Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.hpToClimb),
                            new InstantCommand(() -> {
                                state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when climb is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerLeftDepotClimb extends AutoClass {
        public centerLeftDepotClimb() {
            this.name = "Center Left Depot Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Center to Left Fuel", "Left Fuel to Center", "Center to Depot", "Depot to Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    // new ParallelCommandGroup(
                    //     //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                    //     new WaitCommand(AutosConstants.shootingPause)
                    // ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Left Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToLFuel),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Left Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.lFuelToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Depot")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToDepot),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Depot to Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.depotToClimb),
                            new InstantCommand(() -> {
                                state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when climb is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class leftDepotFuel extends AutoClass {
        public leftDepotFuel() {
            this.name = "Left Depot Fuel (GAME)";
            this.sequentialPathStrings = new String[] { "Left to Center", "Center to Depot", "Depot to Center", "Center to Left Fuel", "Left Fuel to Center"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Left to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.leftToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready

                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Depot")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToDepot),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Depot to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.depotToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Left Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToLFuel),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ), 
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Left Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.lFuelToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerHPFuel extends AutoClass {
        public centerHPFuel() {
            this.name = "Center HP Fuel (GAME)";
            this.sequentialPathStrings = new String[] { "Center to HP", "HP Pickup", "HP to Center", "Center to Right Fuel", "Right Fuel to Center"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to HP")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToHP),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready

                            })
                        )
                    ),
                    AutoBuilder.followPath(pathMap.get("HP Pickup")),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("HP to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.hpToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToRFuel),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ), 
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rFuelToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class rightHPFuel extends AutoClass {
        public rightHPFuel() {
            this.name = "Right HP Fuel (GAME)";
            this.sequentialPathStrings = new String[] { "Right to Center", "Center to Right Fuel", "Right Fuel to Center", "Center to HP", "HP Pickup", "HP to Center"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rightToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready

                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToRFuel),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rightToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ), 
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to HP")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToHP),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    AutoBuilder.followPath(pathMap.get("HP Pickup")),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("HP to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.hpToCenter),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    )
                    ).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class outpostAuto extends AutoClass {
        Pose2d tagPos;

        public outpostAuto() {
            this.name = "Outpost (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Starting to Outpost - AR",
                    "Outpost to 1st Shooting - AR",
                    "1st Shooting to Depot - AR",
                    "Depot to 2nd Shooting - AR"
            };

            tagPos = VisionConstants.kAprilTagLayout.getTagPose(7).get().toPose2d()
                    .plus(new Transform2d(Units.inchesToMeters(36), Units.inchesToMeters(0),
                            new Rotation2d(Units.degreesToRadians(270))));
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        AutoBuilder.followPath(pathMap.get("Starting to Outpost - AR")),
                        // new WaitCommand(2), // Temp seconds amount
                        AutoBuilder.followPath(pathMap.get("Outpost to 1st Shooting - AR")),
                        // ActionCommands.aimAndShoot(state),
                        AutoBuilder.followPath(pathMap.get("1st Shooting to Depot - AR")),
                        // ActionCommands.aimAndShoot(state),
                        AutoBuilder.followPath(pathMap.get("Depot to 2nd Shooting - AR"))
                // new AutoAlignToPoseCommand(state.getDrive(), state, tagPos, 0)
                // ActionCommands.climbUp(state)
                )
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }
    
    public static class depotAuto extends AutoClass {
        Pose2d tagPos;

        public depotAuto() {
            this.name = "Depot (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Starting to Depot - AR",
                    "Depot to 1st Shooting - AR",
                    "1st Shooting to 2nd Shooting - AR"
            };

            tagPos = VisionConstants.kAprilTagLayout.getTagPose(7).get().toPose2d()
                    .plus(new Transform2d(Units.inchesToMeters(36), Units.inchesToMeters(0),
                            new Rotation2d(Units.degreesToRadians(270))));
        }

        @Override
        public Command getCommand(RobotState state) {
            // state.getDrive().setPose(new Pose2d(0, 10, Rotation2d.kZero));
            return ActionCommands.autoClimb(state);
            // try {
            //     Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
            //     new ActionCommands();

            //     return new SequentialCommandGroup(
            //             new InstantCommand(
            //                     () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        
            //             new ParallelCommandGroup(
            //                  AutoBuilder.followPath(pathMap.get("Starting to Depot - AR")),
            //                  new SequentialCommandGroup(
            //                     new WaitCommand(1),
            //                     new InstantCommand(() -> {
            //                         state.getIntake().requestTransition(Intake.State.INTAKE);
            //                     }),

            //                     new WaitCommand(1.5),
            //                     ActionCommands.aimAndShoot(state)
            //                  )
            //             ),
            //             AutoBuilder.followPath(pathMap.get("Depot to 1st Shooting - AR")),
            //             new WaitCommand(1),
            //             new InstantCommand(() -> {
            //                 state.getShooter().requestTransition(State.IDLE);
            //             }),
            //             AutoBuilder.followPath(pathMap.get("1st Shooting to 2nd Shooting - AR")),
            //             new InstantCommand(() -> {
            //                 state.getShooter().requestTransition(State.SHOOTING);
            //                 state.getIntake().requestTransition(Intake.State.STOW);
            //             }),
            //             new WaitCommand(5)
            //     )
            //             .withName(name);
            // } catch (Exception e) {
            //     return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            // }
        }
    }

}
