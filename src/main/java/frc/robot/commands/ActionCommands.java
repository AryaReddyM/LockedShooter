package frc.robot.commands;

import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.FieldConstants;
import frc.robot.util.DynamicPathGenerator;
import frc.robot.util.Util;

public class ActionCommands {
    /*
     * These are where all of our real commands will be.
     * We will test if it's better to have different robotState states or just
     * control it somewhat like this
     * 
     * We will have commands like while B is holding we do aim and Shoot and while
     * it is not holding
     * we will either hub track or pass track depending on our field position
     * estimate
     * 
     */

    // EXAMPLE

    public static PathPlannerPath waypointTestPath() {
        return DynamicPathGenerator.getPathFromWaypoints(
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(),
                        new Pose2d(1, 2, Rotation2d.fromDegrees(90)),
                        new Pose2d(5, 5, Rotation2d.fromDegrees(0))),
                Optional.empty(),
                new GoalEndState(0, new Rotation2d()));
    }

    public static Command aimAndShoot(RobotState state) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> state.getShooter().requestTransition(Shooter.State.HUB_TRACKING)),
                // state.getShooter().getTurret().waitForShootReady(0.1),
                new InstantCommand(() -> state.getShooter().requestTransition(Shooter.State.SHOOTING)));
    }

    public static Command shootOrPassBasedOnPos(RobotState state) {
        return new DeferredCommand(() -> {
            boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
            Pose2d currentPose = state.getLatestFieldToRobot().getValue();

            double blueHubX = VisionConstants.FieldConstants.HUB_BLUE.getX();
            double redHubX = VisionConstants.FieldConstants.HUB_RED.getX();

            boolean shouldShoot = isBlue
                    ? currentPose.getX() <= blueHubX
                    : currentPose.getX() >= redHubX;

            if (shouldShoot && RobotState.hubActivated.get()) {
                return state.getShooter().transitionCommand(Shooter.State.SHOOTING);
            } else {
                return state.getShooter().transitionCommand(Shooter.State.PASSING);
            }
        }, Set.of(state.getShooter()));
    }

    public static Command trackBasedOnPos(RobotState state) {
        return new DeferredCommand(() -> {
            boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
            Pose2d currentPose = state.getLatestFieldToRobot().getValue();

            double blueHubX = VisionConstants.FieldConstants.HUB_BLUE.getX();
            double redHubX = VisionConstants.FieldConstants.HUB_RED.getX();

            boolean shouldShoot = isBlue
                    ? currentPose.getX() <= blueHubX
                    : currentPose.getX() >= redHubX;

            if (shouldShoot && RobotState.hubActivated.get()) {
                return state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING);
            } else {
                return state.getShooter().transitionCommand(Shooter.State.PASS_TRACKING);
            }
        }, Set.of(state.getShooter()));
    }

    public static Command autoClimbV2(RobotState state) {
        return new DeferredCommand(() -> {

            boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

            Translation2d center = VisionConstants.Tower.centerPoint;
            Translation2d left = VisionConstants.Tower.leftUpright;
            Translation2d right = VisionConstants.Tower.rightUpright;

            Pose2d currentPose = state.getLatestFieldToRobot().getValue();

            double robotY = currentPose.getY();
            boolean isLeftSide = robotY > center.getY();

            Translation2d chosenUpright = isLeftSide ? left : right;
            double rotationDeg = isLeftSide ? 180.0 : 0.0;
            double direction = isLeftSide ? 1.0 : -1.0;

            if (!isBlue) {
                chosenUpright = isLeftSide ? right : left;
                chosenUpright = Util.flipRedBlueXY(new Translation3d(chosenUpright)).toTranslation2d();
            }

            Translation2d preClimbTranslation = chosenUpright.plus(new Translation2d(0.0, 0.75 * direction));
            Translation2d climbTranslation = chosenUpright.plus(new Translation2d(0.0, 0.42 * direction));

            Pose2d preClimbPose = new Pose2d(preClimbTranslation, Rotation2d.fromDegrees(rotationDeg));
            Pose2d finalPose = new Pose2d(climbTranslation, Rotation2d.fromDegrees(rotationDeg));

            return new SequentialCommandGroup(
                    state.getIntake().transitionCommand(Intake.State.STOW),
                    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                    new AutoAlignToPoseCommand(state.getDrive(), state, preClimbPose, 1),
                    state.getClimb().transitionCommand(Climb.State.UP),
                    new WaitCommand(0.35),
                    new AutoAlignToPoseCommand(state.getDrive(), state, finalPose, 1),
                    // new ToFAutoAlignToPoseCommand(
                    // state.getDrive(),
                    // state,
                    // finalPose,
                    // 1,
                    // () -> state.getClimb().getLeftSensorDistance(),
                    // () -> state.getClimb().getRightSensorDistance()),
                    state.getClimb().transitionCommand(Climb.State.DOWN));

        }, Set.of(state.getDrive(), state.getClimb(), state.getIntake()));
    }

      public static Command autoClimb(RobotState state) {
        return new DeferredCommand(() -> {

            boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

            Translation2d center = VisionConstants.Tower.centerPoint;
            Translation2d left = VisionConstants.Tower.leftUpright;
            Translation2d right = VisionConstants.Tower.rightUpright;

            Pose2d currentPose = state.getLatestFieldToRobot().getValue();

            double robotY = currentPose.getY();
            boolean isLeftSide = robotY > center.getY();

            Translation2d chosenUpright = isLeftSide ? left : right;
            double rotationDeg = isLeftSide ? 180.0 : 0.0;
            double direction = isLeftSide ? 1.0 : -1.0;

            if (!isBlue) {
                chosenUpright = isLeftSide ? right : left;
                chosenUpright = Util.flipRedBlueXY(new Translation3d(chosenUpright)).toTranslation2d();
            }

            Translation2d preClimbTranslation = chosenUpright.plus(new Translation2d(0.4 * direction, 0.75 * direction));
            Translation2d alignClimbTranslation = chosenUpright.plus(new Translation2d(0.4 * direction, 0.42 * direction));

            Translation2d climbTranslation = chosenUpright.plus(new Translation2d(0.0, 0.42 * direction));

            Pose2d preClimbPose = new Pose2d(preClimbTranslation, Rotation2d.fromDegrees(rotationDeg));
            Pose2d alignClimbPose = new Pose2d(alignClimbTranslation, Rotation2d.fromDegrees(rotationDeg));
            Pose2d finalPose = new Pose2d(climbTranslation, Rotation2d.fromDegrees(rotationDeg));

            return new SequentialCommandGroup(
                    state.getIntake().transitionCommand(Intake.State.STOW),
                    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                    new AutoAlignToPoseCommand(state.getDrive(), state, preClimbPose, 1),
                    state.getClimb().transitionCommand(Climb.State.UP),
                    new WaitCommand(0.35),
                    new AutoAlignToPoseCommand(state.getDrive(), state, alignClimbPose, 1),
                    new AutoAlignToPoseCommand(state.getDrive(), state, finalPose, 1),
                    // new ToFAutoAlignToPoseCommand(
                    // state.getDrive(),
                    // state,
                    // finalPose,
                    // 1,
                    // () -> state.getClimb().getLeftSensorDistance(),
                    // () -> state.getClimb().getRightSensorDistance()),
                    state.getClimb().transitionCommand(Climb.State.DOWN));

        }, Set.of(state.getDrive(), state.getClimb()
        // state.getIntake(), state.getShooter()
        ));
    }

}
