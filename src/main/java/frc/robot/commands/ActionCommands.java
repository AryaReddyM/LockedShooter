package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.FieldConstants;
import frc.robot.util.path.DynamicPathGenerator;
import frc.robot.util.logging.GetTuned;
import frc.robot.util.math.Util;

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

    public static Command shakeIntake(RobotState state) {
        return new RepeatCommand(new SequentialCommandGroup(
            state.getIntake().transitionCommand(Intake.State.SHAKE),
            new WaitCommand(0.6),
            state.getIntake().transitionCommand(Intake.State.IDLE),
            new WaitCommand(0.6)
        ));
    }

    public static Command aimAndShoot(RobotState state) {
        return new SequentialCommandGroup(
                state.getSuperstructure().hubTrack(),
                // state.getShooter().getTurret().waitForShootReady(0.1),
                state.getSuperstructure().shoot());
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
                return state.getSuperstructure().shoot();
            } else {
                return state.getSuperstructure().pass();
            }
        }, Set.of(state.getSuperstructure()));
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

            if (shouldShoot) {
                return state.getSuperstructure().hubTrack();
            } else {
                return state.getSuperstructure().passTrack();
            }
        }, Set.of(state.getSuperstructure()));
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

            Translation2d preClimbTranslation = chosenUpright.plus(new Translation2d(0.0, 0.75 * direction));
            Translation2d climbTranslation = chosenUpright.plus(new Translation2d(0.0, 0.24 * direction));

            Pose2d preClimbPose = new Pose2d(preClimbTranslation, Rotation2d.fromDegrees(rotationDeg));
            Pose2d finalPose = new Pose2d(climbTranslation, Rotation2d.fromDegrees(rotationDeg));

            return new SequentialCommandGroup(
                    state.getIntake().transitionCommand(Intake.State.CLIMB_TOW),
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

        }, Set.of(state.getDrive(), state.getClimb(), state.getIntake(),
        state.getShooter()
        ));
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

            Translation2d preClimbTranslation = chosenUpright.plus(new Translation2d(0.4 * direction, 0.75 * direction));
            Translation2d alignClimbTranslation = chosenUpright.plus(new Translation2d(0.4 * direction, 0.42 * direction));

            Translation2d climbTranslation = chosenUpright.plus(new Translation2d(-0.08 * direction, 0.25 * direction));

            Pose2d preClimbPose = new Pose2d(preClimbTranslation, Rotation2d.fromDegrees(rotationDeg));
            Pose2d alignClimbPose = new Pose2d(alignClimbTranslation, Rotation2d.fromDegrees(rotationDeg));
            Pose2d finalPose = new Pose2d(climbTranslation, Rotation2d.fromDegrees(rotationDeg));

            return new SequentialCommandGroup(
                    state.getIntake().transitionCommand(Intake.State.CLIMB_TOW),
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

        }, Set.of(state.getDrive(), state.getClimb(),
        state.getIntake(),
        state.getShooter()
        ));
    }

    public static Command goToFixedPosAndShoot(RobotState state) {
        return new DeferredCommand(() -> {
            boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
            Pose2d goalPos = VisionConstants.Hub.nearFace.transformBy(new Transform2d(
                new Translation2d(Units.inchesToMeters(80), 0), new Rotation2d()
            ));

            if (!isBlue) {
                goalPos = RobotState.flipPoseForRed(goalPos);
            }

            double turretPos = GetTuned.getNumber("FixedPos/Turret", 3.0301607114946028);

            final double finalPos = turretPos;
            final double shooterRps = GetTuned.getNumber("FixedPos/RPS", 9.8);
            final double hoodPos = GetTuned.getNumber("FixedPos/Hood", 0);
            final double hoodFF = GetTuned.getNumber("FixedPos/Hood FF", 0);
            final double turretFF = GetTuned.getNumber("FixedPos/Turret FF", 0);

            return new SequentialCommandGroup(
                new AutoAlignToPoseCommand(state.getDrive(), state, goalPos, 1),
                new InstantCommand(() -> {
                    
                    state.getShooter().getFlywheel().setOverride(() -> {
                        return shooterRps;
                    });

                    state.getShooter().getHood().setOverride((a) -> {
                        state.getShooter().getHood().setPos(hoodPos,
                        hoodFF);
                    });

                    state.getShooter().getTurret().setOverride((a) -> {
                        state.getShooter().getTurret().setPos(finalPos,
                        turretFF);
                    });
                })

                // new WaitCommand(1),
                // new InstantCommand(() -> {
                //     state.getFuelSim().launchFuel(
                //             MetersPerSecond.of(shooterRps
                //                     * ShooterConstants.kBallLaunchVelMetersPerSecPerRotPerSec),
                //             Degrees.of(90).minus(Radians.of(hoodPos)),
                //             Radians.of(finalPos),
                //             Meters.of(VisionConstants.kTurretToRobotCenter.getTranslation().getZ()));
                // })
            );
        }, Set.of(
            state.getDrive(),
            state.getShooter()
        ));
    }

}
