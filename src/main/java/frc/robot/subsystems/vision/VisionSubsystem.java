package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.util.MathHelpers;
import frc.robot.util.state.StateMachine;

public class VisionSubsystem extends StateMachine<VisionSubsystem.State> {
    private final VisionIO io;
    private final RobotState state;

    private final CameraInputsAutoLogged turretCamera = new CameraInputsAutoLogged();
    private final CameraInputsAutoLogged chassisCamera = new CameraInputsAutoLogged();

    private double lastTurretTimestamp = 0.0;
    private double lastChassisTimestamp = 0.0;

    public enum State {
        UNDETERMINED,
        VISION_SCANNING,
        BROKEN
    }

    public VisionSubsystem(VisionIO io, RobotState state) {
        super("Vision", State.UNDETERMINED, State.class);
        this.io = io;
        this.state = state;
        enable();

        SmartDashboard.putData("Vision Disable", new InstantCommand(() -> {
            state.disableVision();
        }).withName("Vision Disable"));
    }

    @Override
    public void update() {
        io.readInputs(turretCamera, chassisCamera);

        if (getState() == State.BROKEN)
            return;

        // Process both cameras using the robust methods below
        var turretEst = processCamera(turretCamera, "Turret", true);
        var chassisEst = processCamera(chassisCamera, "Chassis", false);

        // Weighted Fusion
        // Optional<VisionFieldPoseEstimate> fused = fuseEstimates(turretEst, chassisEst);

        // if (fused.isPresent()) {
        //     state.updateMegatagEstimate(fused.get());
        // }

        if (turretEst.isPresent()) {
            state.updateMegatagEstimate(turretEst.get());
        }

        if (chassisEst.isPresent()) {
            state.updateMegatagEstimate(chassisEst.get());
        }

        Logger.recordOutput("Chassis Camera Visualization", new Pose3d(state.getLatestFieldToRobot().getValue()).plus(new Transform3d(
            new Translation3d(VisionConstants.kCameraBForwardMeters, // for this, we need to just do it constantly. no biggie
                VisionConstants.kCameraBRightMeters,
                VisionConstants.kCameraBHeightOffGroundMeters), new Rotation3d(
                    Units.degreesToRadians(VisionConstants.kCameraBRollDegrees),
                Units.degreesToRadians(VisionConstants.kCameraBPitchDegrees),
                Units.degreesToRadians(VisionConstants.kCameraBYawDegrees)
                )
        )));
    }

    private Optional<VisionFieldPoseEstimate> processCamera(
            VisionIO.CameraInputs cam, String name, boolean isTurret) {

        if (!cam.seesTarget) {
            return Optional.empty();
        }
        // 1. Pick the best estimate (Prefer Megatag 2)
        boolean useMT2 = cam.megatag2PoseEstimate != null && cam.megatag2Count > 0;
        var estimate = useMT2 ? cam.megatag2PoseEstimate : cam.megatagPoseEstimate;


        
        if (!useMT2 && cam.megatagCount <= 0) {
            return Optional.empty();
        }

        if (estimate == null) {
            return Optional.empty();
        }

        if (estimate.fieldToRobot().equals(Pose2d.kZero)) {
            return Optional.empty();
        }

        if (cam.fiducialObservations.length == 0) {
            return Optional.empty();
        }

        double timestamp = estimate.timestampSeconds();
        String logPath = "Vision/" + name + "/";

        // 2. Timing Check
        if (timestamp == (isTurret ? lastTurretTimestamp : lastChassisTimestamp)) {
            return Optional.empty();
        }

        // 3. Motion Validation (Method fully implemented below)
        if (!isMotionValid(timestamp, isTurret, logPath)) {
            return Optional.empty();
        }

        // 4. Transform Logic (Method fully implemented below)
        Optional<Transform2d> robotToCamera = getRobotToCamera(timestamp, isTurret);
        if (robotToCamera.isEmpty()) {
            return Optional.empty();
        }

        // If Limelight offsets are 0, 'fieldToRobot()' is effectively 'fieldToCamera'
        Pose2d fieldToRobot = estimate.fieldToRobot().plus(robotToCamera.get());

        // 5. Standard Deviation Calculation
        Matrix<N3, N1> stdDevs = calculateStdDevs(cam, estimate, useMT2, isTurret);

        if (isTurret)
            lastTurretTimestamp = timestamp;
        else
            lastChassisTimestamp = timestamp;

        Logger.recordOutput((isTurret ? "Turret" : "Chassis") + " Camera Targets", getCameraAndTagPoses(isTurret));

        return Optional.of(new VisionFieldPoseEstimate(
                fieldToRobot, timestamp, stdDevs, estimate.fiducialIds().length));
    }

    /**
     * Calculates the dynamic transform from the Robot Center to the Camera Lens
     * at a specific point in time using the RobotState history buffers.
     */
    /// TODO fix this based on issues, make sure the megatag of this matches with the chassis
    public Optional<Transform2d> getRobotToCamera(double timestamp, boolean isTurret) {
        if (isTurret) {
            // Retrieve the historical rotation of the turret from the RobotState buffer
            Optional<Rotation2d> turretRotation = state.getRobotToTurret(timestamp);

            if (turretRotation.isPresent()) {
                // Combine: Robot -> Turret (Historical) -> Camera (Static offset on turret)
                Transform2d robotToTurret = MathHelpers.transform2dFromRotation(turretRotation.get().times(-1));
                return Optional.of(robotToTurret.plus(state.getTurretToCamera(true)));
            }
            return Optional.empty();
        }
        // Chassis camera uses a static offset defined in RobotState
        return Optional.of(state.getTurretToCamera(false));
    }

    /**
     * Validates if the robot was stable enough during the frame capture
     * to trust the vision data.
     */
    private boolean isMotionValid(double timestamp, boolean isTurret, String logPath) {
        final double kMaxAngularVel = Units.degreesToRadians(360.0);
        final double kWindow = 0.1; // 100ms window

        // Check Chassis movement
        var chassisVel = state.getMaxAbsDriveYawAngularVelocityInRange(timestamp - kWindow, timestamp);
        double totalAngularVel = Math.abs(chassisVel.orElse(0.0));

        // If it's the turret camera, we must account for the turret's own velocity
        if (isTurret) {
            var turretVel = state.getMaxAbsTurretYawAngularVelocityInRange(timestamp - kWindow, timestamp);
            totalAngularVel = Math.abs(chassisVel.orElse(0.0) + turretVel.orElse(0.0));
        }

        boolean isStable = totalAngularVel < kMaxAngularVel;
        Logger.recordOutput(logPath + "TotalAngularVel", totalAngularVel);
        Logger.recordOutput(logPath + "IsStable", isStable);

        return isStable;
    }

    private Matrix<N3, N1> calculateStdDevs(VisionIO.CameraInputs cam, MegatagPoseEstimate est,
            boolean isMT2, boolean isTurret) {

        int tagCount = isMT2 ? cam.megatag2Count : cam.megatagCount;
        double avgDist = isMT2 ? cam.megatag2avgDist : cam.megatagAvgDist;

        double stdDevFactor =
            Math.pow(avgDist, 2.0) / tagCount;
        double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
        
        if (isMT2) {
          linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
          angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
        }

        int cameraIndex = isTurret ? 1 : 0;
        if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
          linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
          angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
        }

        return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
    }

    private Optional<VisionFieldPoseEstimate> fuseEstimates(
            Optional<VisionFieldPoseEstimate> turret, Optional<VisionFieldPoseEstimate> chassis) {
        if (turret.isEmpty())
            return chassis;
        if (chassis.isEmpty())
            return turret;

        VisionFieldPoseEstimate a = turret.get();
        VisionFieldPoseEstimate b = chassis.get();

        if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
            VisionFieldPoseEstimate tmp = a;
            a = b;
            b = tmp;
        }

        // Preview both estimates to the same timestamp
        Transform2d a_T_b = state.getFieldToRobot(b.getTimestampSeconds())
                .get()
                .minus(state.getFieldToRobot(a.getTimestampSeconds()).get());

        Pose2d poseA = a.getVisionRobotPoseMeters().transformBy(a_T_b);
        Pose2d poseB = b.getVisionRobotPoseMeters();

        // Inverse‑variance weighting
        var varianceA = a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
        var varianceB = b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

        Rotation2d fusedHeading = poseB.getRotation();
        if (varianceA.get(2, 0) < VisionConstants.kLargeVariance
                && varianceB.get(2, 0) < VisionConstants.kLargeVariance) {
            fusedHeading = new Rotation2d(
                    poseA.getRotation().getCos() / varianceA.get(2, 0)
                            + poseB.getRotation().getCos() / varianceB.get(2, 0),
                    poseA.getRotation().getSin() / varianceA.get(2, 0)
                            + poseB.getRotation().getSin() / varianceB.get(2, 0));
        }

        double weightAx = 1.0 / varianceA.get(0, 0);
        double weightAy = 1.0 / varianceA.get(1, 0);
        double weightBx = 1.0 / varianceB.get(0, 0);
        double weightBy = 1.0 / varianceB.get(1, 0);

        Pose2d fusedPose = new Pose2d(
                new Translation2d(
                        (poseA.getTranslation().getX() * weightAx
                                + poseB.getTranslation().getX() * weightBx)
                                / (weightAx + weightBx),
                        (poseA.getTranslation().getY() * weightAy
                                + poseB.getTranslation().getY() * weightBy)
                                / (weightAy + weightBy)),
                fusedHeading);

        Matrix<N3, N1> fusedStdDev = VecBuilder.fill(
                Math.sqrt(1.0 / (weightAx + weightBx)),
                Math.sqrt(1.0 / (weightAy + weightBy)),
                Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

        int numTags = a.getNumTags() + b.getNumTags();
        double time = b.getTimestampSeconds();

        return Optional.of(new VisionFieldPoseEstimate(fusedPose, time, fusedStdDev, numTags));
    }

    public Pose3d[] getCameraAndTagPoses(boolean useTurret) {
        VisionIO.CameraInputs cam = useTurret ? turretCamera : chassisCamera;

        if (!cam.seesTarget) {
            return new Pose3d[0];
        }

        // Prefer MegaTag2
        MegatagPoseEstimate est = cam.megatag2Count > 0 ? cam.megatag2PoseEstimate : cam.megatagPoseEstimate;

        if (est == null || est.fiducialIds() == null || est.fiducialIds().length == 0) {
            return new Pose3d[0];
        }

        // --- Field → Robot ---
        Pose3d fieldToRobot = new Pose3d(est.fieldToRobot());

        // --- Robot → Camera at capture time ---
        Optional<Transform2d> robotToCamera2d = getRobotToCamera(est.timestampSeconds(), useTurret);

        if (robotToCamera2d.isEmpty()) {
            return new Pose3d[0];
        }

        // Transform2d t2d = robotToCamera2d.get();

        // Transform3d robotToCamera = new Transform3d(
        //         new Translation3d(t2d.getX(), t2d.getY(), 0.0),
        //         new Rotation3d(0, 0, t2d.getRotation().getRadians()));

        // --- Field → Camera ---
        Pose3d fieldToCamera = fieldToRobot;//.transformBy(robotToCamera);

        // --- Build camera → tag poses ---
        return java.util.Arrays.stream(est.fiducialIds())
                .mapToObj(id -> {
                var tagPoseOpt = VisionConstants.kAprilTagLayout.getTagPose(id);
                if (tagPoseOpt.isEmpty()) return null;

                Pose3d fieldToTag = tagPoseOpt.get();

                // Camera → Tag
                Pose3d camToTag = fieldToTag.relativeTo(fieldToCamera);

                // Only keep tags within 5 meters
                if (camToTag.getTranslation().getX() > 12) {
                    return null;
                }

                return fieldToTag;
            })
            .filter(java.util.Objects::nonNull)
            .toArray(Pose3d[]::new);
    }

    @Override
    public void determineSelf() {
        setState(State.VISION_SCANNING);
    }
}