package frc.robot.subsystems.kicker;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.GetTuned;
import frc.robot.util.state.StateMachine;

public class Kicker extends StateMachine<Kicker.State> implements KickerIO {

    private final RobotState state;
    private final KickerIO kickerIO;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    private Consumer<Object> override;

    public Kicker(KickerIO kickerIO, RobotState state) {
        super("Kicker", State.UNDETERMINED, State.class);
        this.kickerIO = kickerIO;
        this.state = state;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        kickerIO.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);

        if (override != null) {
            override.accept(null);
        } else if (getState() == State.SHOOT) {
            if (state.getShooter().getFlywheel().isReady()) { // && state.getShooter().getTurret().isReady()) {
                shoot();
            } else {
                stop();
            }
        } else if (getState() == State.OUTAKE) {
            outtake();
        } else {
            stop();
        }

        Logger.recordOutput("Kicker/Overriden", override != null);
    }

    public void shoot() {
        kickerIO.setKickerSpeed(GetTuned.getNumber("Kicker/Shot Speed", KickerConstants.kKickerShootSpeed));

    }

    public void outtake() {
        kickerIO.setKickerSpeed(GetTuned.getNumber("Kicker/Outtake Speed", KickerConstants.kKickerOutakeSpeed));
    }

    public void stop() {
        kickerIO.setKickerSpeed(0);
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.UNDETERMINED, State.IDLE, State.SHOOT, State.OUTAKE);
    }

    private void registerStateCommands() {
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
    }

    public enum State {
        UNDETERMINED,

        IDLE,
        SHOOT,
        OUTAKE

        // flags

    }

}