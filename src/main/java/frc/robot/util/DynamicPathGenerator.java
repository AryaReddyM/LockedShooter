package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;

public class DynamicPathGenerator {
    

    private static PathConstraints getConstraints(boolean f) {
        return DriveConstants.pathConstraint;
    }

    private static PathConstraints getConstraints(Optional<PathConstraints> constraint) {
         PathConstraints pathConstraints;

        if (constraint.isPresent()) {
            pathConstraints = constraint.get();
        } else {
            pathConstraints = getConstraints(true);
        }

        return pathConstraints;
    }

    public static Command pathFindAuto(Pose2d desiredPose, Optional<PathConstraints> constraint) {
        // boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        // if (isRed) {
        //     return AutoBuilder.pathfindToPoseFlipped(desiredPose, pathConstraints);
        // } else {
            return AutoBuilder.pathfindToPose(desiredPose, getConstraints(constraint));
        // }
    }

    public static Command pathfindAuto(PathPlannerPath pathToFollow, Optional<PathConstraints> constraint) {
        return AutoBuilder.pathfindThenFollowPath(pathToFollow, getConstraints(constraint));
    }

    public static PathPlannerPath getPathFromWaypoints(List<Waypoint> waypoints, Optional<PathConstraints> constraint, GoalEndState goalEndState) {
        return new PathPlannerPath(waypoints, getConstraints(constraint), null, goalEndState);
    }

    public static void warmupInit() {
        PathfindingCommand.warmupCommand().schedule();
    }
}
