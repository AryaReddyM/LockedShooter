package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSimPhoton implements VisionIO {

        private static VisionSystemSim visionSim;

        private final PhotonCamera turretCamera = new PhotonCamera(VisionConstants.kLimelightTableName);
        private final PhotonCamera chassisCamera = new PhotonCamera(VisionConstants.kLimelightBTableName);

        private final PhotonCameraSim turretSim;
        private final PhotonCameraSim chassisSim;

        private final Transform3d robotToChassisCam;

        /** Rebuilt every cycle from the turret angle; the turret camera rotates with the turret. */
        private Transform3d robotToTurretCam = new Transform3d();

        private final Supplier<Pose2d> groundTruthPose;
        private final Supplier<Rotation2d> robotToTurret;

        public VisionIOSimPhoton(
                        Supplier<Pose2d> groundTruthPose, Supplier<Rotation2d> robotToTurret) {
                this.groundTruthPose = groundTruthPose;
                this.robotToTurret = robotToTurret;

                if (visionSim == null) {
                        visionSim = new VisionSystemSim("main");
                        visionSim.addAprilTags(VisionConstants.kAprilTagLayout);
                }

                SimCameraProperties props = new SimCameraProperties();
                props.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
                props.setFPS(30);
                props.setAvgLatencyMs(20);

                turretSim = new PhotonCameraSim(turretCamera, props);
                chassisSim = new PhotonCameraSim(chassisCamera, props);

                robotToChassisCam = new Transform3d(
                                new Translation3d(
                                                VisionConstants.kCameraBForwardMeters,
                                                VisionConstants.kCameraBRightMeters,
                                                VisionConstants.kCameraBHeightOffGroundMeters),
                                new Rotation3d(
                                                Units.degreesToRadians(VisionConstants.kCameraBRollDegrees),
                                                -Units.degreesToRadians(VisionConstants.kCameraBPitchDegrees),
                                                Units.degreesToRadians(VisionConstants.kCameraBYawDegrees)));

                visionSim.addCamera(chassisSim, robotToChassisCam);
                visionSim.addCamera(turretSim, robotToTurretCam);
        }

        @Override
        public void readInputs(CameraInputsAutoLogged turretInputs,
                        CameraInputsAutoLogged chassisInputs) {

                // --- 1. Point the turret camera where the turret is now ---
                // This has to happen before the frame is rendered. Rendering first and then
                // moving the camera means the image was taken from the previous turret angle
                // but decoded with the current one, which shows up as a pose error that grows
                // with how fast the turret is moving.
                Rotation2d turretRot = robotToTurret.get();
                if (turretRot != null) {
                        robotToTurretCam = new Transform3d(
                                        new Translation3d(
                                                        VisionConstants.kTurretToCameraX,
                                                        VisionConstants.kTurretToCameraY,
                                                        VisionConstants.kCameraHeightOffGroundMeters),
                                        new Rotation3d(0, 0, turretRot.getRadians()));

                        visionSim.adjustCamera(turretSim, robotToTurretCam);
                }

                // --- 2. Render from the robot's true pose ---
                Pose2d pose = groundTruthPose.get();
                if (pose != null) {
                        visionSim.update(pose);
                }

                // --- 3. Read both cameras ---
                readSimCamera(turretCamera, robotToTurretCam, turretInputs);
                readSimCamera(chassisCamera, robotToChassisCam, chassisInputs);

                Logger.processInputs("Vision/Turret Camera", turretInputs);
                Logger.processInputs("Vision/Chassis Camera", chassisInputs);
        }

        /** Wipes last cycle's observations so a camera that sees nothing does not log stale data. */
        private static void clear(CameraInputsAutoLogged inputs) {
                inputs.seesTarget = false;
                inputs.fiducialObservations = new FiducialObservation[0];
                inputs.megatagPoseEstimate = MegatagPoseEstimate.EMPTY;
                inputs.megatag2PoseEstimate = MegatagPoseEstimate.EMPTY;
                inputs.megatagCount = 0;
                inputs.megatag2Count = 0;
                inputs.megatagAvgDist = 0.0;
                inputs.megatag2avgDist = 0.0;
                inputs.pose3d = new Pose3d();
                inputs.standardDeviations = new double[VisionConstants.kExpectedStdDevArrayLength];
        }

        private void readSimCamera(
                        PhotonCamera camera, Transform3d robotToCamera, CameraInputsAutoLogged inputs) {
                clear(inputs);
                try {
                        List<PhotonPipelineResult> unread = camera.getAllUnreadResults();
                        if (unread.isEmpty()) {
                                return;
                        }
                        PhotonPipelineResult result = unread.get(unread.size() - 1);
                        if (!result.hasTargets()) {
                                return;
                        }

                        List<PhotonTrackedTarget> targets = result.getTargets();
                        int tagCount = targets.size();

                        inputs.seesTarget = true;
                        inputs.megatagCount = tagCount;
                        inputs.megatag2Count = tagCount;

                        inputs.fiducialObservations = targets.stream()
                                        .map(t -> new FiducialObservation(
                                                        t.getFiducialId(),
                                                        t.getYaw(),
                                                        t.getPitch(),
                                                        t.getPoseAmbiguity(),
                                                        t.getArea()))
                                        .toArray(FiducialObservation[]::new);

                        int[] fiducialIds = targets.stream()
                                        .mapToInt(PhotonTrackedTarget::getFiducialId)
                                        .toArray();

                        double avgArea = targets.stream()
                                        .mapToDouble(PhotonTrackedTarget::getArea)
                                        .average()
                                        .orElse(0.0);

                        // Real distance to the tags, which is what drives the standard deviations
                        // downstream. This used to be pinned at 1.0, so a tag across the field was
                        // trusted exactly as much as one a foot away.
                        double avgDist = targets.stream()
                                        .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                                        .average()
                                        .orElse(0.0);

                        // PhotonVision solves for the camera, not the robot. Both solves below end up
                        // as a field-to-camera pose, so the camera mount has to be taken back off
                        // before it can be handed to the pose estimator. For the turret camera that
                        // offset includes the turret's rotation, so skipping it threw the estimate off
                        // by however far the turret happened to be pointed.
                        Pose3d fieldToCamera = null;
                        if (result.getMultiTagResult().isPresent()) {
                                fieldToCamera = new Pose3d()
                                                .transformBy(result.getMultiTagResult().get().estimatedPose.best);
                        } else {
                                PhotonTrackedTarget best = result.getBestTarget();
                                var tagPose = VisionConstants.kAprilTagLayout.getTagPose(best.getFiducialId());
                                if (tagPose.isPresent()) {
                                        fieldToCamera = tagPose.get()
                                                        .transformBy(best.getBestCameraToTarget().inverse());
                                }
                        }

                        if (fieldToCamera == null) {
                                return;
                        }

                        Pose3d fieldToRobot3d = fieldToCamera.transformBy(robotToCamera.inverse());
                        Pose2d fieldToRobot2d = fieldToRobot3d.toPose2d();

                        inputs.pose3d = fieldToRobot3d;
                        inputs.megatagAvgDist = avgDist;
                        inputs.megatag2avgDist = avgDist;

                        MegatagPoseEstimate estimate = new MegatagPoseEstimate(
                                        fieldToRobot2d,
                                        result.getTimestampSeconds(),
                                        result.metadata.getLatencyMillis() / 1000.0,
                                        avgArea,
                                        tagCount * avgArea,
                                        fiducialIds);
                        inputs.megatag2PoseEstimate = estimate;
                        inputs.megatagPoseEstimate = estimate;

                        inputs.standardDeviations = new double[] { 0.1, 0.1, 0, 0, 0, 0.5, 0.1, 0.1, 0, 0, 0, 0.5 };

                } catch (Exception e) {
                        System.err.println("Error processing PhotonVision SIM data: " + e.getMessage());
                }
        }
}
