package frc.robot.subsystems.shooter.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.base.FlywheelMotorSubsystem;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.logging.GetTuned;

public class Flywheel extends FlywheelMotorSubsystem {

    private final RobotState state;
    private double tunedSetpoint = 100.0;
    private double rpsMultiplier = 1.0;
    private boolean ready = false;
    private State stateValue = State.IDLE;

    private Supplier<Double> override;

    public Flywheel(MotorIO io, RobotState state) {
        super(io, "Flywheel", FlywheelConstants.kFlywheelSpeedTolerance);
        this.state = state;

        DogLog.tunable("Flywheel/Custom Setpoint", tunedSetpoint, newSetpoint -> tunedSetpoint = newSetpoint);

        Logger.recordOutput("Flywheel/Multiplier", rpsMultiplier);
        SmartDashboard.putData("Flywheel/Reset Multiplier", new InstantCommand(() -> {
            setMultiplier(1.0);
        }));
    }

    @Override
    public void periodic() {
        super.periodic();

        double desiredRPS = 0;
        { // FLYWHEEL SPEED SETTER
            if (override != null) {
                desiredRPS = override.get();
            } else if (stateValue == State.SHOOT) {
                desiredRPS = state.getCurrentHubSetpoint().getShooterRPS() * rpsMultiplier;
            } else if (stateValue == State.PASS) {
                desiredRPS = state.getCurrentPassSetpoint().getShooterRPS();
            } else if (stateValue == State.TUNING) {
                desiredRPS = tunedSetpoint;
            } else if (stateValue == State.TRACKING) {
                desiredRPS = FlywheelConstants.kSlowSpeed;
            } else {
                desiredRPS = 0;
            }
        }

        shoot(desiredRPS);
        ready = computeReady(desiredRPS,
                GetTuned.getNumber("Flywheel/Speed Tolerance", FlywheelConstants.kFlywheelSpeedTolerance));
        Logger.recordOutput("Flywheel/Ready", ready);
        Logger.recordOutput("Flywheel/Overriden", override != null);
        Logger.recordOutput("Flywheel/State", stateValue.toString());
    }

    private boolean computeReady(double desiredRPS, double tolerance) {
        if (desiredRPS < 1) {
            return false;
        }
        return (desiredRPS - tolerance) < inputs.velocityRadPerSec;
    }

    public void shoot(double speed, double ff) {
        applySetpoint(Setpoint.motionMagicVelocity(speed), ff);
    }

    public void shoot(double speed) {
        applySetpoint(Setpoint.motionMagicVelocity(speed));
    }

    public boolean isReady() {
        return ready;
    }

    public void setOverride(Supplier<Double> override) {
        this.override = override;
    }

    public void setMultiplier(double newMultiplier) {
        rpsMultiplier = newMultiplier;
        Logger.recordOutput("Flywheel/Multiplier", newMultiplier);
    }

    public double getMultiplier() {
        return rpsMultiplier;
    }

    public State getState() {
        return stateValue;
    }

    public void requestTransition(State state) {
        stateValue = state == State.UNDETERMINED ? State.IDLE : state;
    }

    public Command transitionCommand(State state) {
        return runOnce(() -> requestTransition(state));
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
