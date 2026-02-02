package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSpark;
import frc.robot.util.state.StateMachine;

public class Shooter extends StateMachine<Shooter.State> {
    
    private Turret turret;
    private Hood hood;
    private Flywheel flywheel;


    public Shooter(RobotState state) {
        super("Shooter", State.UNDETERMINED, State.class);

        turret = new Turret(new TurretIOSpark(state), state);
        hood = new Hood(); // TODO
        flywheel = new Flywheel(new FlywheelIOSpark()); // TODO

        registerStateTransitions();
        registerStateCommands();

        addChildSubsystem(turret);
        // addChildSubsystem(hood); // TODO
        addChildSubsystem(flywheel);
        enable();
    }

    public void registerStateTransitions() {
        addOmniTransitions(State.UNDETERMINED, State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.SHOOTING, State.PASSING);
    }

    public void registerStateCommands() {
        registerStateCommand(State.IDLE, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.IDLE);
            // TODO hood n flywheel
        }));

        registerStateCommand(State.HUB_TRACKING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.HUB_TRACKING);
            // TODO flywheel
        }));

        registerStateCommand(State.PASS_TRACKING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.PASS_TRACKING);
            // TODO flywheel
        }));

        registerStateCommand(State.SHOOTING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.HUB_TRACKING);
            // TODO flywheel
        }));

        registerStateCommand(State.PASSING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.PASS_TRACKING);
            // TODO flywheel
        }));
    }

    public enum State {
        UNDETERMINED,

        IDLE,
        HUB_TRACKING,
        PASS_TRACKING,
        SHOOTING,
        PASSING

        // flags

    }

    @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
    }
}
