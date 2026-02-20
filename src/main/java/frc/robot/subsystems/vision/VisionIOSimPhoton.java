package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.util.SimulatedRobotState;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOSimPhoton extends VisionIOHardwareLimelight {

    private static VisionSystemSim visionSim;

    private final PhotonCamera turretCamera =
            new PhotonCamera(VisionConstants.kLimelightTableName);
    private final PhotonCamera chassisCamera =
            new PhotonCamera(VisionConstants.kLimelightBTableName);

    private final PhotonCameraSim turretSim;
    private final PhotonCameraSim chassisSim;

    private final SimulatedRobotState simState;
    private final RobotState robotState;

    public VisionIOSimPhoton(RobotState state, SimulatedRobotState simState) {
        super(state);
        this.robotState = state;
        this.simState = simState;

        // Create vision system once
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.kAprilTagLayout);
        }

        // Camera properties (match Limelight-ish behavior)
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 800, Rotation2d.fromDegrees(97));
        props.setFPS(30);
        props.setAvgLatencyMs(20);

        turretSim = new PhotonCameraSim(turretCamera, props);
        chassisSim = new PhotonCameraSim(chassisCamera, props);

        // Static chassis camera
        Transform3d robotToChassisCam = new Transform3d(
                new Translation3d(
                        VisionConstants.kCameraBForwardMeters,
                        VisionConstants.kCameraBRightMeters,
                        VisionConstants.kCameraBHeightOffGroundMeters),
                new Rotation3d(
                        0,
                        -Units.degreesToRadians(VisionConstants.kCameraBPitchDegrees),
                        Units.degreesToRadians(VisionConstants.kCameraBYawDegrees))
        );

        visionSim.addCamera(chassisSim, robotToChassisCam);

        // Turret camera starts at origin (updated dynamically)
        visionSim.addCamera(turretSim, new Transform3d());
    }

    @Override
    public void readInputs(CameraInputsAutoLogged turretInputs,
                           CameraInputsAutoLogged chassisInputs) {

        // --- 1. Update robot pose ---
        Pose2d pose = simState.getLatestFieldToRobot();
        if (pose != null) {
            visionSim.update(pose);
        }

        // --- 2. Update turret transform dynamically ---
        var turretRotEntry = robotState.getLatestRobotToTurret();
        if (turretRotEntry != null) {
            Rotation2d turretRot = turretRotEntry.getValue();

            Transform3d robotToTurretCam = new Transform3d(
                    new Translation3d(
                            VisionConstants.kTurretToCameraX,
                            VisionConstants.kTurretToCameraY,
                            VisionConstants.kCameraHeightOffGroundMeters),
                    new Rotation3d(0, 0, turretRot.getRadians())
            );

            visionSim.adjustCamera(turretSim, robotToTurretCam);
        }

        // --- 3. Push Photon results into Limelight NetworkTables ---
        writeLatestResultToLimelight(
                turretCamera,
                NetworkTableInstance.getDefault()
                        .getTable(VisionConstants.kLimelightTableName));

        writeLatestResultToLimelight(
                chassisCamera,
                NetworkTableInstance.getDefault()
                        .getTable(VisionConstants.kLimelightBTableName));

        // --- 4. Let your existing Limelight pipeline handle everything ---
        super.readInputs(turretInputs, chassisInputs);
    }

    private void writeLatestResultToLimelight(PhotonCamera camera, NetworkTable table) {
        PhotonPipelineResult result = camera.getLatestResult();

        table.getEntry("tv").setDouble(result.hasTargets() ? 1 : 0);
        if (!result.hasTargets()) return;

        if (result.getMultiTagResult().isPresent()) {
            var multi = result.getMultiTagResult().get();

            if (multi.estimatedPose != null) {
                Transform3d pose = multi.estimatedPose.best;

                // Limelight botpose_wpiblue format
                double[] botpose = new double[] {
                        pose.getX(),
                        pose.getY(),
                        pose.getZ(),
                        Units.radiansToDegrees(pose.getRotation().getX()),
                        Units.radiansToDegrees(pose.getRotation().getY()),
                        Units.radiansToDegrees(pose.getRotation().getZ()),
                        0, // latency
                        multi.fiducialIDsUsed.size(),
                        0,0,0
                };

                table.getEntry("botpose_wpiblue").setDoubleArray(botpose);

                // Fake MT2 stddevs
                table.getEntry("stddevs").setDoubleArray(
                        new double[] {0.1,0.1,0,0,0,0.5, 0.1,0.1,0,0,0,0.5});
            }
        }
    }
}