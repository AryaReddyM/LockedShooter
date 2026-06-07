package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.util.sim.SimulatedRobotState;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOSimPhoton implements VisionIO {

        private static VisionSystemSim visionSim;

        private final PhotonCamera turretCamera = new PhotonCamera(VisionConstants.kLimelightTableName);
        private final PhotonCamera chassisCamera = new PhotonCamera(VisionConstants.kLimelightBTableName);

        private final PhotonCameraSim turretSim;
        private final PhotonCameraSim chassisSim;

        private final SimulatedRobotState simState;
        private final RobotState robotState;

        public VisionIOSimPhoton(RobotState state, SimulatedRobotState simState) {
                this.robotState = state;
                this.simState = simState;

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

                Transform3d robotToChassisCam = new Transform3d(
                                new Translation3d(
                                                VisionConstants.kCameraBForwardMeters,
                                                VisionConstants.kCameraBRightMeters,
                                                VisionConstants.kCameraBHeightOffGroundMeters),
                                new Rotation3d(
                                                Units.degreesToRadians(VisionConstants.kCameraBRollDegrees),
                                                -Units.degreesToRadians(VisionConstants.kCameraBPitchDegrees),
                                                Units.degreesToRadians(VisionConstants.kCameraBYawDegrees)));

                visionSim.addCamera(chassisSim, robotToChassisCam);
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
                                        new Rotation3d(0, 0, turretRot.getRadians()));

                        visionSim.adjustCamera(turretSim, robotToTurretCam);
                }

                // --- 3. Read turret camera ---
                readSimCamera(turretCamera, turretInputs);

                // --- 4. Read chassis camera ---
                readSimCamera(chassisCamera, chassisInputs);

                Logger.processInputs("Vision/Turret Camera", turretInputs);
                Logger.processInputs("Vision/Chassis Camera", chassisInputs);
        }

        private void readSimCamera(PhotonCamera camera, CameraInputsAutoLogged inputs) {
                try {
                        var result = camera.getLatestResult();

                        inputs.seesTarget = result.hasTargets();
                        if (!inputs.seesTarget) {
                                return;
                        }

                        int tagCount = result.getTargets().size();
                        inputs.megatag2Count = tagCount;
                        inputs.megatagCount = tagCount;

                        // --- Fiducial observations (Limelight-style) ---
                        inputs.fiducialObservations = result.getTargets().stream()
                                        .map(t -> new FiducialObservation(
                                                        t.getFiducialId(),
                                                        t.getYaw(),
                                                        t.getPitch(),
                                                        t.getPoseAmbiguity(),
                                                        t.getArea()))
                                        .toArray(FiducialObservation[]::new);

                        // --- Build fiducial ID array ---
                        int[] fiducialIds = result.getTargets().stream()
                                        .mapToInt(t -> t.getFiducialId())
                                        .toArray();

                        // --- Average tag area ---
                        double avgArea = result.getTargets().stream()
                                        .mapToDouble(t -> t.getArea())
                                        .average()
                                        .orElse(0.0);


                        // --- Quality heuristic (MegaTag2-like) ---
                        double quality = tagCount * avgArea;

                        Pose2d fieldToRobot2d = null;
                        Pose3d fieldToRobot3d = null;

                        // --- Multi-tag solve (MegaTag2 equivalent) ---
                        if (result.getMultiTagResult().isPresent()) {
                                var multi = result.getMultiTagResult().get();

                                // Photon gives field→camera transform
                                Transform3d fieldToCamera = multi.estimatedPose.best;

                                Pose3d fieldToCameraPose = new Pose3d().transformBy(fieldToCamera);

                                // Convert camera pose → robot pose using sim camera transform
                                // PhotonCameraSim already uses the robot-relative transform,
                                // so fieldToCamera is effectively fieldToRobot if using VisionSystemSim
                                fieldToRobot3d = fieldToCameraPose;
                                fieldToRobot2d = fieldToRobot3d.toPose2d();
                        }
                        // --- Fallback: single tag ---
                        else {
                                var best = result.getBestTarget();
                                var tagPoseOpt = VisionConstants.kAprilTagLayout.getTagPose(best.getFiducialId());

                                if (tagPoseOpt.isPresent()) {
                                        fieldToRobot3d = tagPoseOpt.get()
                                                        .transformBy(best.getBestCameraToTarget().inverse());
                                        fieldToRobot2d = fieldToRobot3d.toPose2d();
                                }
                        }

                        if (fieldToRobot2d != null) {
                                inputs.pose3d = fieldToRobot3d;

                                double timestamp = result.getTimestampSeconds();
                                double latency = 20 / 1000.0;

                                inputs.megatag2PoseEstimate = new MegatagPoseEstimate(
                                                fieldToRobot2d,
                                                timestamp,
                                                latency,
                                                avgArea,
                                                quality,
                                                fiducialIds);
                                inputs.megatag2avgDist = 1.0;
                                inputs.megatagAvgDist = 1.0;

                                // Mirror for compatibility
                                inputs.megatagPoseEstimate = inputs.megatag2PoseEstimate;
                        }

                        inputs.standardDeviations = new double[] { 0.1, 0.1, 0, 0, 0, 0.5, 0.1, 0.1, 0, 0, 0, 0.5 };

                } catch (Exception e) {
                        System.err.println("Error processing PhotonVision SIM data: " + e.getMessage());
                }
        }
}