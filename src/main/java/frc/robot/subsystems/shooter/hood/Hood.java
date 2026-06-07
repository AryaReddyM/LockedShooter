package frc.robot.subsystems.shooter.hood;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.ServoMotorSubsystem;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.field.TrenchZone;

public class Hood extends ServoMotorSubsystem {
    private final RobotState state;
    private double tunedSetpoint = 0.0;
    private double desiredPos = 0.0;
    private Consumer<Object> override;
    private State stateValue = State.IDLE;

    public Hood(MotorIO io, RobotState state) {
        super(io, "Hood", HoodConstants.kHoodDeviationErr);

        DogLog.tunable("Hood/Custom Setpoint", tunedSetpoint, newSetpoint -> tunedSetpoint = newSetpoint);

        this.state = state;
    }

    public void setPos(double position, double ff) {
        if (TrenchZone.hoodLowerRequired(state) && position > HoodConstants.kHoodMaxSetpointUnderTrench) {
            position = HoodConstants.kHoodMaxSetpointUnderTrench;
        }
        desiredPos = position;
        applySetpoint(Setpoint.motionMagicPosition(position), ff);
    }

    @Override
    public void stop() {
        super.stop();
    }

    public double getHoodPosition() {
        return desiredPos;
    }

    public Command waitForShootReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentHubSetpoint().getHoodRadians() - getPositionRad()) < tolerance;
        });
    }

    public Command waitForPassReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentPassSetpoint().getHoodRadians() - getPositionRad()) < tolerance;
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        { // HOOD POS SETTER

            if (TrenchZone.hoodLowerRequired(state) && desiredPos > HoodConstants.kHoodMaxSetpointUnderTrench) {
                setPos(HoodConstants.kHoodMaxSetpointUnderTrench, 0);
            }

            if (override != null) {
                override.accept(null);
            } else if (stateValue == State.HUB_TRACKING) {
                setPos(state.getCurrentHubSetpoint().getHoodRadians(), state.getCurrentHubSetpoint().getHoodFF());
            } else if (stateValue == State.PASS_TRACKING) {
                setPos(state.getCurrentPassSetpoint().getHoodRadians(), state.getCurrentPassSetpoint().getHoodFF());
            } else if (stateValue == State.TUNING) {
                setPos(tunedSetpoint, 0);
            } else {
                stop();
            }
        }

        Logger.recordOutput("Hood/Desired", desiredPos);
        Logger.recordOutput("Hood/Pose",
                new Pose3d()
                        .plus(VisionConstants.kTurretToRobotCenter)
                        .plus(new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0, 0, state.getTurretDesiredPositionRad())))
                        .plus(HoodConstants.turretToHood)
                        .plus(new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0, Units.degreesToRadians(-120) + desiredPos, 0))));
        Logger.recordOutput("Hood/Overriden", override != null);
        Logger.recordOutput("Hood/State", stateValue.toString());
    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
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
        PASS_TRACKING,
        HUB_TRACKING,
        TUNING
    }
}
