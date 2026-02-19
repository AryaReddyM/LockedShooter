package frc.robot.subsystems.climb;

import frc.robot.RobotState;

public class BeamBreakerSim implements BeamBreakerIO {
    private double distance = 0.0;

    public BeamBreakerSim(int num, RobotState state) {
    }

    @Override
    public void updateInputs(BeamBreakerInputs inputs) {
        inputs.distance = 0;
        this.distance = inputs.distance;
    }

    @Override
    public double getDistance() {
        return distance;
    }
}