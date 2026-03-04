package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;

public class TrenchZone {

    // meters
    static double HOOD_LOWER_RADIUS = 0.6;
    static double INTAKE_LOWER_RADIUS = 1.0;
    

    public static double getDistanceToClosestTrench(RobotState state) {
        Pose2d robotPose = state.getLatestFieldToRobot().getValue();
        Translation2d robotTranslation = robotPose.getTranslation();

        double bottomTrenchY = VisionConstants.FieldConstants.TRENCH_CENTER.in(Meters);
        double topTrenchY = VisionConstants.FieldConstants.FIELD_WIDTH.in(Meters)
                - VisionConstants.FieldConstants.TRENCH_CENTER.in(Meters);

        double allianceTrenchXCenter = VisionConstants.FieldConstants.TRENCH_BUMP_X.in(Meters);
        double opponentTrenchXCenter = VisionConstants.FieldConstants.FIELD_LENGTH.in(Meters)
                - VisionConstants.FieldConstants.TRENCH_BUMP_X.in(Meters);

        Translation2d[] trenchCenters = {
                new Translation2d(allianceTrenchXCenter, bottomTrenchY),
                new Translation2d(allianceTrenchXCenter, topTrenchY),
                new Translation2d(opponentTrenchXCenter, bottomTrenchY),
                new Translation2d(opponentTrenchXCenter, topTrenchY)
        };

        double minDistance = Double.MAX_VALUE;
        for (Translation2d trenchPoint : trenchCenters) {
            double distance = robotTranslation.getDistance(trenchPoint);
            if (distance < minDistance) {
                minDistance = distance;
            }
        }

        return minDistance;
    }

    public static double getDistanceToClosestShootingPose(RobotState state) {
        Pose2d robotPose = state.getLatestFieldToRobot().getValue();
        Translation2d robotTranslation = robotPose.getTranslation();

        double blueHub = VisionConstants.kBlueHubPose.toTranslation2d().getDistance(robotTranslation);
        double redHub = VisionConstants.kRedHubPose.toTranslation2d().getDistance(robotTranslation);

        return Math.min(blueHub, redHub);
    }

    public static boolean intakeLowerRequired(RobotState state) {
        return getDistanceToClosestTrench(state) < INTAKE_LOWER_RADIUS;
    }

    public static boolean driveRotationOverrideRequired(RobotState state) {
        return intakeLowerRequired(state);
    }

    public static boolean hoodLowerRequired(RobotState state) {
        return getDistanceToClosestTrench(state) < HOOD_LOWER_RADIUS;
    }

}
