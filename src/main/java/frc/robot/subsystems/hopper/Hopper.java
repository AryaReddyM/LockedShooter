package frc.robot.subsystems.hopper;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.base.FlywheelMotorSubsystem;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.logging.GetTuned;

public class Hopper extends FlywheelMotorSubsystem {

    private State stateValue = State.IDLE;

    private double spinRadians = 0.0;

    private Consumer<Object> override;

    public Hopper(MotorIO io) {
        super(io, "Hopper", HopperConstants.kHopperDeviationErr);
    }

    @Override
    public void periodic() {
        super.periodic();

        spinRadians = spinRadians + (0.02 * HopperConstants.kHopperShootSpeed / HopperConstants.kRollerRadiusMeters);

        Logger.recordOutput("Hopper/Pose",
                new Pose3d()
                        .plus(HopperConstants.hopperOrigin)
                        .plus(new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0, 0, spinRadians))));

        if (override != null) {
            override.accept(null);
        } else if (stateValue == State.SHOOT) {
            shoot();
        } else if (stateValue == State.OUTAKE) {
            outake();
        } else {
            stop();
        }
        Logger.recordOutput("Hopper/Overriden", override != null);
        Logger.recordOutput("Hopper/State", stateValue.toString());
    }

    public void shoot() {
        applySetpoint(Setpoint.motionMagicVelocity(
                GetTuned.getNumber("Hopper/Shoot Speed", HopperConstants.kHopperShootSpeed)));
    }

    public void outake() {
        applySetpoint(Setpoint.motionMagicVelocity(
                GetTuned.getNumber("Hopper/Outtake Speed", HopperConstants.kHopperOuttakeSpeed)));
    }

    @Override
    public void stop() {
        applySetpoint(Setpoint.motionMagicVelocity(0.0));
    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
    }

    public State getState() {
        return stateValue;
    }

    public void requestTransition(State state) {
        stateValue = state == State.UNDETERMINED ? State.IDLE : state;
    }

    public Command transitionCommand(State state) {
        return runOnce(() -> requestTransition(state));
    }

    public enum State {
        UNDETERMINED,

        IDLE,
        OUTAKE,
        SHOOT

        // flags

    }

}
