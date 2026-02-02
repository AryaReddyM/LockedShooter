package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.util.state.StateMachine;

public class Hood extends StateMachine<Hood.State> {
    private final RobotState state;
    private final HoodIO hoodIO;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO hoodIO, RobotState state) {
        super("Hood", State.UNDETERMINED, State.class);

        this.state = state;
        this.hoodIO = hoodIO;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    public void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.UNDETERMINED);
    }

    public void registerStateCommands() {

    }

    public void setPos(double position, double ff) {
        hoodIO.setHoodPosition(position, ff);
    }

    public void stop() {
        hoodIO.stopHood();
    }

    public Command waitForShootReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentHubSetpoint().getHoodRadians() - hoodIO.getHoodPosition()) < tolerance;
        });
    }

    public Command waitForPassReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentPassSetpoint().getHoodRadians() - hoodIO.getHoodPosition()) < tolerance;
        });
    }



    @Override
    public void update() {
        hoodIO.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        { // HOOD POS SETTER
            if (getState() == State.HUB_TRACKING) {
                setPos(state.getCurrentHubSetpoint().getHoodRadians(), state.getCurrentHubSetpoint().getHoodFF());
            } else if(getState() == State.PASS_TRACKING) {
                setPos(state.getCurrentPassSetpoint().getHoodRadians(), state.getCurrentPassSetpoint().getHoodFF());
            } else {
                stop();
            }
        }
    }

    @Override
    public void determineSelf() {
        setState(State.UNDETERMINED);
    }
    
    public enum State {
        UNDETERMINED,

        IDLE,
        PASS_TRACKING,
        HUB_TRACKING,
    }
}
