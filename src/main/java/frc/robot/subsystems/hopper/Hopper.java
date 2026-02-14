package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.GetTuned;
import frc.robot.util.state.StateMachine;

public class Hopper extends StateMachine<Hopper.State> implements HopperIO{

    private final RobotState state;
    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public Hopper(HopperIO hopperIO, RobotState state) {
        super("Hopper", State.UNDETERMINED, State.class);
        this.hopperIO = hopperIO;
        this.state = state;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        hopperIO.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    public void shoot() {
        hopperIO.setHopperSpeed(GetTuned.getNumber("Hopper/Shoot Speed", HopperConstants.kHopperShootSpeed));
    }

    public void outake() {
        hopperIO.setHopperSpeed(GetTuned.getNumber("Hopper/Outtake Speed", HopperConstants.kHopperOuttakeSpeed));
    }

    public void stop() {
        hopperIO.stopHopper();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.OUTAKE, State.SHOOT);
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, Commands.run(() -> stop()));
        registerStateCommand(State.OUTAKE, Commands.run(() -> outake()));
        registerStateCommand(State.SHOOT, Commands.run(() -> shoot()));
    }

     @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,

        IDLE,
        OUTAKE,
        SHOOT

        // flags

    }
    
}
