package frc.robot.subsystems.climb;

public class BeamBreakerIOSim implements BeamBreakerIO {
  private static final double kNoTargetDistance = 2.0;

  @Override
  public void updateInputs(BeamBreakerInputs inputs) {
    inputs.distance = kNoTargetDistance;
  }

  @Override
  public double getDistance() {
    return kNoTargetDistance;
  }
}
