package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Shooter;

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
    public static Command aimAndShoot(RobotState state) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> state.getShooter().requestTransition(Shooter.State.HUB_TRACKING)),
                state.getShooter().getTurret().waitForShootReady(0.1),
                new InstantCommand(() -> state.getShooter().requestTransition(Shooter.State.SHOOTING)));
    }

    public static Command aimAtProperItem(RobotState state) {
        return new InstantCommand(() -> {
            Pose2d robotPose = state.getLatestFieldToRobot().getValue();

            // some checks here to see if we are in OUR side or not
            var alliance = DriverStation.getAlliance();
            boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

            // Logic: If Red, "our area" is the right side (high X). If Blue, left side (low
            // X).
            // Adjust the 8.75 threshold based on where the 2026 "midline" actually is.
            boolean inOurArea = isRed ? (robotPose.getX() > 8.75) : (robotPose.getX() < 8.75);
            /// THIS IS AI AND IT WONT WORK PROBABLY (TEMPLATE CODE ^)

            state.getShooter().requestTransition(inOurArea ? Shooter.State.HUB_TRACKING : Shooter.State.PASS_TRACKING);
        });
    }

}
