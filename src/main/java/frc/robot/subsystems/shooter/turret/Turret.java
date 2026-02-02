package frc.robot.subsystems.shooter.turret;


import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.util.RobotTime;
import frc.robot.util.ShooterSetpoint;
import frc.robot.util.state.StateMachine;

public class Turret extends StateMachine<Turret.State> implements TurretIO{
    private final RobotState state;
    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO turretIO, RobotState state) {
        super("Turret", State.UNDETERMINED, State.class);
        this.turretIO = turretIO;
        this.state = state;

        registerStateTransitions();
        registerStateCommands();
        enable();
    }


    @Override
    public void update() {
        turretIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        
        state.addTurretUpdates(RobotTime.getTimestampSeconds(), inputs.turretRotation2d,
                inputs.turretPos,
                inputs.turretVelRadPerSec);
    }

    public void setPos(double position, double ff) {
        turretIO.setTurretPosition(position, ff);
    }

    public void stop() {
        turretIO.stopTurret();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING);
    }

    private void registerStateCommands() {
        setDefaultCommand(new InstantCommand( () -> {
            if (getState() == State.HUB_TRACKING) {
                setPos(state.getCurrentHubSetpoint().getTurretRadiansFromCenter(), state.getCurrentHubSetpoint().getTurretFF());
            } else if (getState() == State.PASS_TRACKING) {
                setPos(state.getCurrentHubSetpoint().getTurretRadiansFromCenter(), state.getCurrentHubSetpoint().getTurretFF());
            } else {
                stop();
            }
        }));
    }

     @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,

        IDLE,
        HUB_TRACKING,
        PASS_TRACKING

        // flags

    }

    
    
}
