public class autos {

    public static class leftDepotClimb extends AutoClass {
        public leftDepotClimb() {
            this.name = "Left Depot & Climb (GAME)";
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
                            new WaitCommand(1.5),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Depot")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.7),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Depot to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.5),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.8),
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
            this.sequentialPathStrings = new String[] { "Right to Center", "Center to Fuel", "Left Fuel to Center", "Center to Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.5),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready//state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready

                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Fuel")),
                        new SequentialCommandGroup(
                            new WaitCommand(2.5),
                            new InstantCommand(() -> {
                                //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Left Fuel to Center")),
                        new SequentialCommandGroup(
                            new WaitCommand(3.5),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Climb")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.9),
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
