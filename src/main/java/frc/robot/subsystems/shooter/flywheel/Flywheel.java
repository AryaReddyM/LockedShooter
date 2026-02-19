package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import frc.robot.RobotState;
import frc.robot.util.state.StateMachine;

public class Flywheel extends StateMachine<Flywheel.State> implements FlywheelIO{

    private final RobotState state;
    private final FlywheelIO flywheelIO;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private double tunedSetpoint = 0.0;

    public Flywheel(FlywheelIO flywheelIO, RobotState state) {
        super("Flywheel", State.UNDETERMINED, State.class);
        this.flywheelIO = flywheelIO;
        this.state = state;

        DogLog.tunable("Flywheel/Custom Setpoint", tunedSetpoint, newSetpoint -> tunedSetpoint = newSetpoint);

        registerStateTransitions();
        registerStateCommands();
        enable();
    }


    @Override
    public void update() {
        flywheelIO.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        
        { // FLYWHEEL SPEED SETTER
            if (getState() == State.SHOOT) {
                shoot(state.getCurrentHubSetpoint().getShooterRPS());
            } else if(getState() == State.PASS) {
                shoot(state.getCurrentPassSetpoint().getShooterRPS());
            } else if (getState() == State.TUNING) {
                shoot(tunedSetpoint);
            } else if(getState() == State.TRACKING){
                slow();
            } else {
                stop();
            }
        }
    }

    public void shoot(double pos, double ff) {
        flywheelIO.setFlywheelSpeed(pos, ff);

    }

    public void shoot(double pos) {
        flywheelIO.setFlywheelSpeed(pos);
    }

    public void stop() {
        flywheelIO.stopFlywheel();
    }

    public void slow() {
        flywheelIO.setFlywheelSpeed(FlywheelConstants.kSlowSpeed);
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.SHOOT, State.PASS, State.UNDETERMINED, State.TRACKING);
    }

    private void registerStateCommands() {
    }

     @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
    }
    
    public enum State {
        UNDETERMINED,

        IDLE,
        SHOOT,
        PASS,
        TRACKING,
        TUNING

        // flags

    }
    
}
