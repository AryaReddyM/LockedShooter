package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

public class Shooter extends SubsystemBase {
    
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final RobotState state;
    private State stateValue = State.IDLE;

    public Shooter(RobotState state, MotorIO turretIO, MotorIO hoodIO, MotorIO flywheelIO) {
        turret = new Turret(turretIO, state);
        hood = new Hood(hoodIO, state);
        flywheel = new Flywheel(flywheelIO, state);
        this.state = state;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/State", stateValue.toString());
    }

    public void requestTransition(State newState) {
        stateValue = newState == State.UNDETERMINED ? State.IDLE : newState;
        switch (stateValue) {
            case IDLE:
                turret.requestTransition(Turret.State.IDLE);
                flywheel.requestTransition(Flywheel.State.IDLE);
                hood.requestTransition(Hood.State.IDLE);
                break;
            case HUB_TRACKING:
                turret.requestTransition(Turret.State.HUB_TRACKING);
                flywheel.requestTransition(Flywheel.State.TRACKING);
                hood.requestTransition(Hood.State.HUB_TRACKING);
                break;
            case PASS_TRACKING:
                turret.requestTransition(Turret.State.PASS_TRACKING);
                flywheel.requestTransition(Flywheel.State.TRACKING);
                hood.requestTransition(Hood.State.PASS_TRACKING);
                break;
            case SHOOTING:
                turret.requestTransition(Turret.State.HUB_TRACKING);
                flywheel.requestTransition(Flywheel.State.SHOOT);
                hood.requestTransition(Hood.State.HUB_TRACKING);
                break;
            case PASSING:
                turret.requestTransition(Turret.State.PASS_TRACKING);
                flywheel.requestTransition(Flywheel.State.PASS);
                hood.requestTransition(Hood.State.PASS_TRACKING);
                break;
            case OUTTAKE:
                turret.requestTransition(Turret.State.IDLE);
                flywheel.requestTransition(Flywheel.State.IDLE);
                hood.requestTransition(Hood.State.IDLE);
                break;
            case TUNING:
                turret.requestTransition(Turret.State.TUNING);
                flywheel.requestTransition(Flywheel.State.TUNING);
                hood.requestTransition(Hood.State.TUNING);
                break;
            case UNDETERMINED:
                break;
        }
    }

    public Command transitionCommand(State state) {
        return runOnce(() -> requestTransition(state));
    }

    public State getState() {
        return stateValue;
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
        OUTTAKE,
        TUNING,

        // flags

    }
}
