package frc.robot.subsystems.autos;

public class autos {

    public static class leftDepotClimb extends AutoClass {
        public leftDepotClimb() {
            this.name = "Left Depot Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Left to Center", "Center to Depot", "Depot to Center", "Center to Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Left to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(leftToCenter),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Depot")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToDepot),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Depot to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(depotToCenter),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToClimb),
                            new InstantCommand(() -> {
                                //state.getClimb().requestTransition(State.CLIMBING); // uncomment when climb is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }
    
    public static class rightClimb extends AutoClass {
        public rightClimb() {
            this.name = "Right Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Right to Center", "Center to Right Fuel", "Right Fuel to Center", "Center to Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(rightToCenter),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready//state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready

                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToRFuel),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(rFuelToCenter),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToClimb),
                            new InstantCommand(() -> {
                                //state.getClimb().requestTransition(State.CLIMBING); // uncomment when climb is ready
                            })
                        )
                    )
                    )
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
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
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    AutoBuilder.followPath(pathMap.get("Center to HP")),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("HP Pickup")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToClimb),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("HP to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(hpToCenter),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToClimb),
                            new InstantCommand(() -> {
                                //state.getClimb().requestTransition(State.CLIMBING); // uncomment when climb is ready
                            })
                        )
                    )
                    )
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
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
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                        new WaitCommand(shootingPause)
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToRFuel),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(rFuelToCenter),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to HP")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToHP),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    AutoBuilder.followPath(pathMap.get("HP Pickup")),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("HP to Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(hpToClimb),
                            new InstantCommand(() -> {
                                //state.getClimb().requestTransition(State.CLIMBING); // uncomment when climb is ready
                            })
                        )
                    )
                )
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }

    //Not Done
    public static class centerRightHPClimb extends AutoClass {
        public centerRightHPClimb() {
            this.name = "Center Right HP Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Center to Right Fuel", "Right Fuel to Center", "Center to HP", "HP Pickup"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                        new WaitCommand(shootingPause)
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToRFuel),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(rFuelToCenter),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to HP")),
                        new SequentialCommandGroup(
                            new WaitCommand(centerToHP),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to HP")),
                        new SequentialCommandGroup(
                            new WaitCommand(hpToClimb),
                            new InstantCommand(() -> {
                                //state.getClimb().requestTransition(State.CLIMBING); // uncomment when climb is ready
                            })
                        )
                    )
                )
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }


}
