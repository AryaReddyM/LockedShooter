package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;

public class Superstructure extends SubsystemBase {

    public enum State {
        IDLE,
        TRAVERSING,
        HUB_TRACKING,
        PASS_TRACKING,
        INTAKING,
        OUTTAKING,
        SHOOTING,
        SHOOTING_INTAKING,
        PASSING,
        PASSING_INTAKING,
        CLIMBING
    }

    private final Intake intake;
    private final Shooter shooter;
    private final Climb climb;
    private final Hopper hopper;
    private final Kicker kicker;

    private State state = State.IDLE;

    public Superstructure(Intake intake, Shooter shooter, Climb climb, Hopper hopper, Kicker kicker) {
        this.intake = intake;
        this.shooter = shooter;
        this.climb = climb;
        this.hopper = hopper;
        this.kicker = kicker;
    }

    public void setState(State newState) {
        state = newState;
        switch (newState) {
            case IDLE:
                shooter.requestTransition(Shooter.State.IDLE);
                intake.requestTransition(Intake.State.IDLE);
                climb.requestTransition(Climb.State.STOW);
                hopper.requestTransition(Hopper.State.IDLE);
                kicker.requestTransition(Kicker.State.IDLE);
                break;
            case TRAVERSING:
            case HUB_TRACKING:
                shooter.requestTransition(Shooter.State.HUB_TRACKING);
                intake.requestTransition(Intake.State.STOW);
                hopper.requestTransition(Hopper.State.IDLE);
                kicker.requestTransition(Kicker.State.IDLE);
                break;
            case PASS_TRACKING:
                shooter.requestTransition(Shooter.State.PASS_TRACKING);
                intake.requestTransition(Intake.State.STOW);
                hopper.requestTransition(Hopper.State.IDLE);
                kicker.requestTransition(Kicker.State.IDLE);
                break;
            case INTAKING:
                shooter.requestTransition(Shooter.State.HUB_TRACKING);
                intake.requestTransition(Intake.State.INTAKE);
                hopper.requestTransition(Hopper.State.IDLE);
                kicker.requestTransition(Kicker.State.IDLE);
                break;
            case OUTTAKING:
                shooter.requestTransition(Shooter.State.OUTTAKE);
                intake.requestTransition(Intake.State.OUTAKE);
                hopper.requestTransition(Hopper.State.OUTAKE);
                kicker.requestTransition(Kicker.State.OUTAKE);
                break;
            case SHOOTING:
                shooter.requestTransition(Shooter.State.SHOOTING);
                intake.requestTransition(Intake.State.STOW);
                hopper.requestTransition(Hopper.State.SHOOT);
                kicker.requestTransition(Kicker.State.SHOOT);
                break;
            case SHOOTING_INTAKING:
                shooter.requestTransition(Shooter.State.SHOOTING);
                intake.requestTransition(Intake.State.INTAKE);
                hopper.requestTransition(Hopper.State.SHOOT);
                kicker.requestTransition(Kicker.State.SHOOT);
                break;
            case PASSING:
                shooter.requestTransition(Shooter.State.PASSING);
                intake.requestTransition(Intake.State.STOW);
                hopper.requestTransition(Hopper.State.SHOOT);
                kicker.requestTransition(Kicker.State.SHOOT);
                break;
            case PASSING_INTAKING:
                shooter.requestTransition(Shooter.State.PASSING);
                intake.requestTransition(Intake.State.INTAKE);
                hopper.requestTransition(Hopper.State.SHOOT);
                kicker.requestTransition(Kicker.State.SHOOT);
                break;
            case CLIMBING:
                shooter.requestTransition(Shooter.State.IDLE);
                intake.requestTransition(Intake.State.CLIMB_TOW);
                climb.requestTransition(Climb.State.UP);
                hopper.requestTransition(Hopper.State.IDLE);
                kicker.requestTransition(Kicker.State.IDLE);
                break;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Superstructure/State", state.toString());
    }

    public State getState() {
        return state;
    }

    public Command toState(State requested) {
        return runOnce(() -> setState(requested));
    }

    public Command idle() {
        return toState(State.IDLE);
    }

    public Command intake() {
        return toState(State.INTAKING);
    }

    public Command hubTrack() {
        return toState(State.HUB_TRACKING);
    }

    public Command passTrack() {
        return toState(State.PASS_TRACKING);
    }

    public Command outtake() {
        return toState(State.OUTTAKING);
    }

    public Command shoot() {
        return toState(State.SHOOTING);
    }

    public Command shootWhileIntaking() {
        return toState(State.SHOOTING_INTAKING);
    }

    public Command pass() {
        return toState(State.PASSING);
    }

    public Command climb() {
        return toState(State.CLIMBING);
    }
}
