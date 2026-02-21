package frc.robot.commands.autos;

public class Autos {

     public static class testAuto extends AutoClass {
        public leftToFuelToClimbAuto() {
            this.name = "Left to Fuel to Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Left to Fuel",  "Center to Climb"};
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Left to Fuel")),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    // state.getShooter().requestTransition(State.SHOOTING);
                                }))),
                        new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Center to Climb")),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {
                                    // state.getShooter().requestTransition(State.SHOOTING);
                                })))
                                
                                
                                
                                )
                
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }
    
}
