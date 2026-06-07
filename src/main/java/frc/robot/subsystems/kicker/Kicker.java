package frc.robot.subsystems.kicker;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.base.FlywheelMotorSubsystem;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.logging.GetTuned;

public class Kicker extends FlywheelMotorSubsystem {

    private State stateValue = State.IDLE;

    private Consumer<Object> override;

    public Kicker(MotorIO io) {
        super(io, "Kicker", KickerConstants.kKickerDeviationErr);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (override != null) {
            override.accept(null);
        } else if (stateValue == State.SHOOT) {
            shoot();
        } else if (stateValue == State.OUTAKE) {
            outtake();
        } else {
            stop();
        }

        Logger.recordOutput("Kicker/Overriden", override != null);
        Logger.recordOutput("Kicker/State", stateValue.toString());
    }

    public void shoot() {
        applySetpoint(Setpoint.motionMagicVelocity(
                GetTuned.getNumber("Kicker/Shot Speed", KickerConstants.kKickerShootSpeed)));
    }

    public void outtake() {
        applySetpoint(Setpoint.motionMagicVelocity(
                GetTuned.getNumber("Kicker/Outtake Speed", KickerConstants.kKickerOutakeSpeed)));
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
        SHOOT,
        OUTAKE

        // flags

    }

}
