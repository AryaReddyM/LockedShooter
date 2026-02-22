package frc.robot.subsystems.shooter.flywheel;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import frc.robot.RobotState;
import frc.robot.util.state.StateMachine;
import frc.robot.util.GetTuned;

public class Flywheel extends StateMachine<Flywheel.State> implements FlywheelIO{

    private final RobotState state;
    private final FlywheelIO flywheelIO;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private double tunedSetpoint = 0.0;

    private Supplier<Double> override;

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
        
        double desiredRPS = 0;
        { // FLYWHEEL SPEED SETTER
            if (override != null) {
                desiredRPS = override.get();
            }else if (getState() == State.SHOOT) {
                desiredRPS = state.getCurrentHubSetpoint().getShooterRPS();
            } else if(getState() == State.PASS) {
                desiredRPS = state.getCurrentPassSetpoint().getShooterRPS();
            } else if (getState() == State.TUNING) {
                desiredRPS = tunedSetpoint;
            } else if(getState() == State.TRACKING){
                desiredRPS = FlywheelConstants.kSlowSpeed;
            } else {
                desiredRPS = 0;
            }
        }

        shoot(desiredRPS);
        inputs.isReady = flywheelIO.isAtSpeed(desiredRPS, GetTuned.getNumber("Flywheel/Speed Tolerance", FlywheelConstants.kFlywheelSpeedTolerance));
    }

    public void shoot(double pos, double ff) {
        flywheelIO.setFlywheelSpeed(pos, ff);

    }

    public void shoot(double pos) {
        flywheelIO.setFlywheelSpeed(pos);
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.SHOOT, State.PASS, State.UNDETERMINED, State.TRACKING);
    }

    private void registerStateCommands() {
    }

    public boolean isReady() {
        return inputs.isReady;
    }
     @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
    }

    public void setOverride(Supplier<Double> override) {
        this.override = override;
    }
    
    public enum State {
        UNDETERMINED,

        IDLE,
        SHOOT,
        PASS,
        TRACKING,
        TUNING,

        // flags

    }
    
}
