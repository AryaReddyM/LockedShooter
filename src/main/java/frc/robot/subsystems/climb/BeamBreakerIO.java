package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakerIO {
    
    @AutoLog
    public static class BeamBreakerInputs{
        public double distance = 0.0;
    }

    default void updateInputs(BeamBreakerInputs inputs) {}
    default double getDistance() {return 0.0;};
}
