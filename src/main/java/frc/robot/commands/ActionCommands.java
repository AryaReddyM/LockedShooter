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

    public static PathPlannerPath startOwnCenterToDepot() {
        return DynamicPathGenerator.getPathFromWaypoints(
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(4, 4, Rotation2d.fromDegrees(180)),
                        new Pose2d(0.5, 6, Rotation2d.fromDegrees(180))),
                Optional.empty(),
                new GoalEndState(0, Rotation2d.fromDegrees(180)));
    }

    public static PathPlannerPath depotToHang() {
        return DynamicPathGenerator.getPathFromWaypoints(
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(0.5, 6, Rotation2d.fromDegrees(180)),
                        new Pose2d(0.5, 4.5, Rotation2d.fromDegrees(90))),
                Optional.empty(),
                new GoalEndState(0, Rotation2d.fromDegrees(90)));
    }

    public static Command aimAndShoot(RobotState state) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> state.getShooter().requestTransition(Shooter.State.HUB_TRACKING)),
                // state.getShooter().getTurret().waitForShootReady(0.1),
                new InstantCommand(() -> state.getShooter().requestTransition(Shooter.State.SHOOTING)));
    }

    public static Command aimAtProperItem(RobotState state) {
        return new InstantCommand(() -> {
            // Pose2d robotPose = state.getLatestFieldToRobot().getValue();

            // some checks here to see if we are in OUR side or not
            // var alliance = DriverStation.getAlliance();
            // boolean isRed = alliance.isPresent() && alliance.get() ==
            // DriverStation.Alliance.Red;

            // Logic: If Red, "our area" is the right side (high X). If Blue, left side (low
            // X).
            // Adjust the 8.75 threshold based on where the 2026 "midline" actually is.
            // boolean inOurArea = isRed ? (robotPose.getX() > 8.75) : (robotPose.getX() <
            // 8.75);
            /// THIS IS AI AND IT WONT WORK PROBABLY (TEMPLATE CODE ^)

            boolean inOurArea = true;

            state.getShooter().requestTransition(inOurArea ? Shooter.State.HUB_TRACKING : Shooter.State.PASS_TRACKING);
        });
    }

    public static Command climbUp(RobotState state) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> state.getClimb().requestTransition(Climb.State.UP)),
                new InstantCommand(() -> state.getClimb().requestTransition(Climb.State.IDLE)));
    }

    public static Command climbDown(RobotState state) {
        return new InstantCommand(() -> state.getClimb().requestTransition(Climb.State.CLIMB));
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
                chosenUpright = isLeftSide ? right: left;
                chosenUpright = Util.flipRedBlueXY(new Translation3d(chosenUpright)).toTranslation2d();
            }

            Translation2d preClimbTranslation = chosenUpright.plus(new Translation2d(0.0, 0.75 * direction));
            Translation2d climbTranslation = chosenUpright.plus(new Translation2d(0.0, 0.44 * direction));

            Pose2d preClimbPose = new Pose2d(preClimbTranslation, Rotation2d.fromDegrees(rotationDeg));
            Pose2d finalPose = new Pose2d(climbTranslation, Rotation2d.fromDegrees(rotationDeg));

            return new SequentialCommandGroup(
                    new AutoAlignToPoseCommand(state.getDrive(), state, preClimbPose, 1),
                    state.getClimb().transitionCommand(Climb.State.UP),
                    new WaitCommand(0.5),
                    new AutoAlignToPoseCommand(state.getDrive(), state, finalPose, 1),
                    // new ToFAutoAlignToPoseCommand(
                    //         state.getDrive(),
                    //         state,
                    //         finalPose,
                    //         1,
                    //         () -> state.getClimb().getLeftSensorDistance(),
                    //         () -> state.getClimb().getRightSensorDistance()),
                    state.getClimb().transitionCommand(Climb.State.DOWN));

        }, Set.of(state.getDrive()));
    }

}
