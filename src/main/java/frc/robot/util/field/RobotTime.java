package frc.robot.util.field;

import edu.wpi.first.wpilibj.Timer;

public class RobotTime {
    public static double getTimestampSeconds() {
        return Timer.getFPGATimestamp();
    }
}