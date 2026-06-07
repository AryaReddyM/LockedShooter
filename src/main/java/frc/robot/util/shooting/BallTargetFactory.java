package frc.robot.util.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;

import org.littletonrobotics.junction.Logger;

public class BallTargetFactory {

    static InterpolatingTreeMap<Double, Double> heightMap = new InterpolatingTreeMap<Double, Double>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static { // TODO fix (distance meters, vertical offsetMeters)
        double scale = 0.5; // scaling factor to raise shot to hub height

        heightMap.put(5.34, 5.34 * Math.tan(Math.toRadians(27)) * scale); // ~3.82
        heightMap.put(4.90, 4.90 * Math.tan(Math.toRadians(26)) * scale); // ~3.35
        heightMap.put(4.44, 4.44 * Math.tan(Math.toRadians(25.5)) * scale); // ~2.97
        heightMap.put(4.05, 4.05 * Math.tan(Math.toRadians(25)) * scale); // ~2.65
        heightMap.put(3.74, 3.74 * Math.tan(Math.toRadians(24)) * scale); // ~2.34
        heightMap.put(3.42, 3.42 * Math.tan(Math.toRadians(23)) * scale); // ~2.03
        heightMap.put(3.06, 3.06 * Math.tan(Math.toRadians(22)) * scale); // ~1.74
        heightMap.put(2.73, 2.73 * Math.tan(Math.toRadians(20.5)) * scale); // ~1.43
        heightMap.put(2.45, 2.45 * Math.tan(Math.toRadians(19.5)) * scale); // ~1.22
        heightMap.put(2.14, 2.14 * Math.tan(Math.toRadians(18)) * scale); // ~0.98
        heightMap.put(1.86, 1.86 * Math.tan(Math.toRadians(17)) * scale); // ~0.80
        heightMap.put(1.55, 1.55 * Math.tan(Math.toRadians(15)) * scale); // ~0.58

    }
    static InterpolatingTreeMap<Double, Double> distanceOffsetMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static { // TODO fix (distance meters, lateral offset)
        distanceOffsetMap.put(1.4, Units.inchesToMeters(0.0));
        distanceOffsetMap.put(3.0, Units.inchesToMeters(0.0));
    }

    static Double kXDistanceOffset = Units.inchesToMeters(0);

    public static Translation3d generate(RobotState robotState) {
        // uncomment to calibrate shooter to center of the face above the goal
        var speakerPose = robotState.isRedAlliance() ? VisionConstants.kRedHubPose
                : VisionConstants.kBlueHubPose;

        double distance = new Translation2d(speakerPose.getX(), speakerPose.getY()).getDistance(
                robotState.getLatestFieldToRobot().getValue().getTranslation());

        double distanceOffset = distanceOffsetMap.get(distance);
        // Do math in blue alliance, we flip for red.
        var offSet = new Translation2d(kXDistanceOffset, -distanceOffset);

        if (robotState.isRedAlliance()) {
            offSet = new Translation2d(-offSet.getX(), offSet.getY());
        }

        Logger.recordOutput("BallTargetFactory/distanceFromTarget", distance);
        speakerPose = new Translation3d(
                speakerPose.getX() + offSet.getX(), speakerPose.getY() + offSet.getY(),
                speakerPose.getZ() + heightMap.get(distance));

        Logger.recordOutput("targetPose", speakerPose);
        Logger.recordOutput("targetPose2d", new Translation2d(speakerPose.getX(), speakerPose.getY()));
        return speakerPose;
    }
}