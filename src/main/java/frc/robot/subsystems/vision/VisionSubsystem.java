package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.state.StateMachine;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends StateMachine<VisionSubsystem.State> {
  private final VisionIO io;
  private final Drive drive;

  private final CameraInputsAutoLogged turretCamera = new CameraInputsAutoLogged();
  private final CameraInputsAutoLogged chassisCamera = new CameraInputsAutoLogged();

  private double lastTurretTimestamp = 0.0;
  private double lastChassisTimestamp = 0.0;

  private static final double kMaxYawRateRadPerSec = Units.degreesToRadians(360.0);

  public VisionSubsystem(VisionIO io, Drive drive) {
    super("Vision", State.UNDETERMINED, State.class);
    this.io = io;
    this.drive = drive;

    addOmniTransitions(State.VISION_SCANNING, State.BROKEN);
  }

  @Override
  protected void update() {
    io.readInputs(turretCamera, chassisCamera);

    if (getState() == State.BROKEN) {
      return;
    }

    processCamera(turretCamera, "Turret", true).ifPresent(this::addToDrive);
    processCamera(chassisCamera, "Chassis", false).ifPresent(this::addToDrive);
  }

  private void addToDrive(VisionFieldPoseEstimate est) {
    drive.addVisionMeasurement(
        est.getVisionRobotPoseMeters(),
        est.getTimestampSeconds(),
        est.getVisionMeasurementStdDevs());
  }

  private Optional<VisionFieldPoseEstimate> processCamera(
      VisionIO.CameraInputs cam, String name, boolean isTurret) {
    if (!cam.seesTarget || cam.megatagCount <= 0) {
      return Optional.empty();
    }

    MegatagPoseEstimate estimate = cam.megatagPoseEstimate;
    if (estimate == null
        || estimate.fieldToRobot() == null
        || estimate.fieldToRobot().equals(Pose2d.kZero)
        || cam.fiducialObservations.length == 0) {
      return Optional.empty();
    }

    double timestamp = estimate.timestampSeconds();
    double lastTimestamp = isTurret ? lastTurretTimestamp : lastChassisTimestamp;
    if (timestamp == lastTimestamp) {
      return Optional.empty();
    }

    double yawRate = Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond);
    Logger.recordOutput("Vision/" + name + "/YawRate", yawRate);
    if (yawRate > kMaxYawRateRadPerSec) {
      return Optional.empty();
    }

    Matrix<N3, N1> stdDevs = calculateStdDevs(cam, isTurret);

    if (isTurret) {
      lastTurretTimestamp = timestamp;
    } else {
      lastChassisTimestamp = timestamp;
    }

    return Optional.of(
        new VisionFieldPoseEstimate(
            estimate.fieldToRobot(), timestamp, stdDevs, estimate.fiducialIds().length));
  }

  private Matrix<N3, N1> calculateStdDevs(VisionIO.CameraInputs cam, boolean isTurret) {
    double stdDevFactor = Math.pow(cam.megatagAvgDist, 2.0) / Math.max(cam.megatagCount, 1);
    double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
    double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

    int cameraIndex = isTurret ? 1 : 0;
    if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
      linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
      angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
    }
    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  public void disableVision() {
    requestTransition(State.BROKEN);
  }

  @Override
  protected void determineSelf() {
    setState(State.VISION_SCANNING);
  }

  public enum State {
    UNDETERMINED,
    VISION_SCANNING,
    BROKEN
  }
}
