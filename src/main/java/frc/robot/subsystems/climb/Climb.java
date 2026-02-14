package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.GetTuned;
import frc.robot.util.state.StateMachine;

public class Climb extends StateMachine<Climb.State> implements ClimbIO{

    private final RobotState state;
    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO climbIO, RobotState state) {
        super("Climb", State.UNDETERMINED, State.class);
        this.climbIO = climbIO;
        this.state = state;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        climbIO.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public void stow() {
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Stow Setpoint", ClimbConstants.kClimbStowPos));
    }
    
    public void up() {
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Up Setpoint", ClimbConstants.kClimbUpPos));

    }

    public void down(){
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Down Setpoint", ClimbConstants.kClimbDownPos));

    }

    public void stop() {
        climbIO.stopClimb();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.STOW, State.IDLE, State.UP, State.CLIMB);
    }

    private void registerStateCommands() {
        registerStateCommand(State.STOW, Commands.run(() -> stow()));
        registerStateCommand(State.IDLE, Commands.run(() -> stop()));
        registerStateCommand(State.UP, Commands.run(() -> up()));
        registerStateCommand(State.CLIMB, Commands.run(() -> down()));
    }

     @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,

        STOW,
        IDLE,
        UP,
        CLIMB

        // flags

    }
    
}
