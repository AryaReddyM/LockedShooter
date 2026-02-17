package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.RobotTime;
import frc.robot.util.TurretVisualizer;
import frc.robot.util.state.StateMachine;

public class Turret extends StateMachine<Turret.State> implements TurretIO {
    private final RobotState state;
    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretVisualizer turretVisualizer;
    private final Timer timer;

    public Turret(TurretIO turretIO, RobotState state) {
        super("Turret", State.UNDETERMINED, State.class);
        this.turretIO = turretIO;
        this.state = state;

        this.timer = new Timer();
        this.turretVisualizer = new TurretVisualizer(() -> {
            return inputs.robotTurretPos;
        }, () -> {return state.getLatestMeasuredFieldRelativeChassisSpeeds();});

        timer.start();

        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        turretIO.updateInputs(inputs);
        {
            Rotation2d robotToTurrRot = inputs.turretRotation2d;

            if (state.robotState == 2) {
                robotToTurrRot = Rotation2d.fromRadians(inputs.desiredPos);

                turretVisualizer.updateFuel(
                    MetersPerSecond.of(state.getCurrentHubSetpoint().getShooterRPS() * ShooterConstants.kBallLaunchVelMetersPerSecPerRotPerSec), 
                    Degrees.of(90).minus(Radians.of(state.getCurrentHubSetpoint().getHoodRadians())));

                if (state.getShooter().getState() == Shooter.State.SHOOTING && state.getSimFuelCount() > 0 && (timer.hasElapsed(0.08))) {
                    state.setSimFuelCount(state.getSimFuelCount()-1);
                    timer.reset();

                    state.getFuelSim().launchFuel(MetersPerSecond.of(state.getCurrentHubSetpoint().getShooterRPS() * ShooterConstants.kBallLaunchVelMetersPerSecPerRotPerSec),
                            Degrees.of(90).minus(Radians.of(state.getCurrentHubSetpoint().getHoodRadians())),
                            Radians.of(state.getCurrentHubSetpoint().getTurretRadiansFromCenter()),
                            Inches.of(state.getCurrentHubSetpoint().getHeight()));
                }
            }
            inputs.robotTurretPos = new Pose3d(
                    state.getLatestFieldToRobot().getValue().getX(),
                    state.getLatestFieldToRobot().getValue().getY(),
                    0.0,
                    new Rotation3d(0.0, 0.0, state.getLatestFieldToRobot().getValue().getRotation().plus(robotToTurrRot).getRadians()));
        }
        Logger.processInputs("Turret", inputs);

        { // TURRET POS SETTER
            if (getState() == State.HUB_TRACKING) {
                setPos(state.getCurrentHubSetpoint().getTurretRadiansFromCenter(),
                        state.getCurrentHubSetpoint().getTurretFF());
            } else if (getState() == State.PASS_TRACKING) {
                setPos(state.getCurrentPassSetpoint().getTurretRadiansFromCenter(),
                        state.getCurrentPassSetpoint().getTurretFF());
            } else {
                stop();
            }
        }

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
        addOmniTransitions(State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.UNDETERMINED);
    }

    private void registerStateCommands() {
    }

    public Command waitForShootReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentHubSetpoint().getTurretRadiansFromCenter()
                    - turretIO.getTurretPosition()) < tolerance;
        });
    }

    public Command waitForPassReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentPassSetpoint().getTurretRadiansFromCenter()
                    - turretIO.getTurretPosition()) < tolerance;
        });
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