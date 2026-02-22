package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.util.state.StateMachine;

public class Shooter extends StateMachine<Shooter.State> {
    
    private Turret turret;
    private Hood hood;
    private Flywheel flywheel;


    public Shooter(RobotState state, TurretIO turretIO, HoodIO hoodIO, FlywheelIO flywheelIO) {
        super("Shooter", State.UNDETERMINED, State.class);

        turret = new Turret(turretIO, state);
        hood = new Hood(hoodIO, state);
        flywheel = new Flywheel(flywheelIO, state);

        registerStateTransitions();
        registerStateCommands();

        addChildSubsystem(turret);
        addChildSubsystem(hood);
        addChildSubsystem(flywheel);
        enable();
    }

    public void registerStateTransitions() {
        addOmniTransitions(State.UNDETERMINED, State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.SHOOTING, State.PASSING);
    }

    public void registerStateCommands() {
        registerStateCommand(State.IDLE, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.IDLE);
            flywheel.requestTransition(Flywheel.State.IDLE);
            hood.requestTransition(Hood.State.IDLE);
        }));

        registerStateCommand(State.HUB_TRACKING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.HUB_TRACKING);
            flywheel.requestTransition(Flywheel.State.TRACKING);
            hood.requestTransition(Hood.State.HUB_TRACKING);
        }));

        registerStateCommand(State.PASS_TRACKING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.PASS_TRACKING);
            flywheel.requestTransition(Flywheel.State.TRACKING);
            hood.requestTransition(Hood.State.PASS_TRACKING);
        }));

        registerStateCommand(State.SHOOTING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.HUB_TRACKING);
            flywheel.requestTransition(Flywheel.State.SHOOT);
            hood.requestTransition(Hood.State.HUB_TRACKING);
        }));

        registerStateCommand(State.PASSING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.PASS_TRACKING);
            flywheel.requestTransition(Flywheel.State.PASS);
            hood.requestTransition(Hood.State.PASS_TRACKING);
        }));

        registerStateCommand(State.TUNING, new InstantCommand(() -> {
            turret.requestTransition(Turret.State.TUNING);
            flywheel.requestTransition(Flywheel.State.TUNING);
            hood.requestTransition(Hood.State.TUNING);
        }));
    }

    public Turret getTurret() {
        return turret;
    }

    public Hood getHood() {
        return hood;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public enum State {
        UNDETERMINED,

        IDLE,
        HUB_TRACKING,
        PASS_TRACKING,
        SHOOTING,
        PASSING,
        TUNING,

        // flags

    }

    @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
    }
}
