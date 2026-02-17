package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.util.RobotTime;
import frc.robot.util.state.StateMachine;

public class Turret extends StateMachine<Turret.State> implements TurretIO {
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
        {
            Rotation2d robotToTurrRot = inputs.turretRotation2d;

            if (state.robotState == 2) {
                robotToTurrRot = Rotation2d.fromRadians(inputs.desiredPos);
            }
            inputs.robotTurretPos = new Pose3d(
                    state.getLatestFieldToRobotCenter().getX(),
                    state.getLatestFieldToRobotCenter().getY(),
                    0.0,
                    new Rotation3d(0.0, 0.0, state.getLatestFieldToRobotCenter().getRotation().plus(robotToTurrRot).getRadians()));
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