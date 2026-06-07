package frc.robot.subsystems.climb;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.ServoMotorSubsystem;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.logging.Elastic;
import frc.robot.util.logging.Elastic.Notification;
import frc.robot.util.logging.GetTuned;

public class Climb extends ServoMotorSubsystem {

    private final RobotState state;
    private final BeamBreakerInputsAutoLogged leftInputs = new BeamBreakerInputsAutoLogged();
    private final BeamBreakerInputsAutoLogged rightInputs = new BeamBreakerInputsAutoLogged();
    private final LoggedMechanism2d climbMechanism = new LoggedMechanism2d(3, 4);

    private final BeamBreakerIO leftSensor;
    private final BeamBreakerIO rightSensor;

    private final LoggedMechanismLigament2d climbElevatorExtension;
    private Consumer<Object> override;
    private State stateValue = State.IDLE;

    private double desiredPos = 0.0;

    public Climb(MotorIO io, BeamBreakerIO leftSensor, BeamBreakerIO rightSensor, RobotState state) {
        super(io, "Climb", ClimbConstants.kClimbDeviationErr);
        this.state = state;
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;

        LoggedMechanismRoot2d climbRoot = climbMechanism.getRoot("Climber", 1.85, 0);
        LoggedMechanismLigament2d climbElevatorBase = climbRoot
                .append(new LoggedMechanismLigament2d("elevator", ClimbConstants.kClimberBaseHeight, 90));
        climbElevatorExtension = climbElevatorBase
                .append(new LoggedMechanismLigament2d("extension", 0, 0, 10, new Color8Bit(255, 0, 0)));

        SmartDashboard.putData("Climb Zero", zero().withName("Climb Zero"));
    }

    @Override
    public void periodic() {
        super.periodic();

        leftSensor.updateInputs(leftInputs);
        rightSensor.updateInputs(rightInputs);

        Logger.processInputs("Left Sensor", leftInputs);
        Logger.processInputs("Right Sensor", rightInputs);

        climbElevatorExtension.setLength(desiredPos);
        Logger.recordOutput("Climb/Mechanism", climbMechanism);

        Logger.recordOutput("Climb/Pose",
                new Pose3d()
                        .plus(new Transform3d()).plus(
                                new Transform3d(
                                        new Translation3d(0, 0,
                                                desiredPos * ClimbConstants.kClimbPositionConversionFactor),
                                        new Rotation3d())));

        if (override != null) {
            override.accept(null);
        } else if (stateValue == State.STOW) {
            stow();
        } else if (stateValue == State.IDLE) {
            stop();
        } else if (stateValue == State.UP) {
            up();
        } else if (stateValue == State.DOWN) {
            down();
        }
        Logger.recordOutput("Climb/Overriden", override != null);
        Logger.recordOutput("Climb/State", stateValue.toString());
    }

    public void stow() {
        desiredPos = GetTuned.getNumber("Climb/Stow Setpoint", ClimbConstants.kClimbStowPos);
        applySetpoint(Setpoint.motionMagicPosition(desiredPos));
    }

    public void up() {
        desiredPos = GetTuned.getNumber("Climb/Up Setpoint", ClimbConstants.kClimbUpPos);
        applySetpoint(Setpoint.motionMagicPosition(desiredPos));
    }

    public void down() {
        desiredPos = GetTuned.getNumber("Climb/Down Setpoint", ClimbConstants.kClimbDownPos);
        io.setMotionMagicPosition(desiredPos, 1); // slot 1 = softer "action" gains
    }

    public void stop() {
        io.stop();
    }

    public Command zero() {
        return run(() -> {
            io.setDutyCycle(GetTuned.getNumber("Climb/Lower Motor Output", ClimbConstants.kLowerMotorOutput));
        })
                .beforeStarting(() -> {
                    io.setCurrentLimit(
                            GetTuned.getNumber("Climb/Lower Current Limit", ClimbConstants.kLowerCurrentLimit));
                })
                .until(() -> inputs.currentAmps > GetTuned.getNumber("Climb/Zero Current Threshold",
                        ClimbConstants.kZeroCurrentThreshold))
                .finallyDo(interrupted -> {
                    io.stop();
                    io.setEncoderPosition(0);
                    io.setCurrentLimit(ClimbConstants.kClimbCurrentLimit);
                    requestTransition(State.STOW);
                    Elastic.sendNotification(
                            new Notification().withTitle("Climb Zero").withDescription("Climb has been zeroed!"));
                });
    }

    public double getLeftSensorDistance() {
        return leftSensor.getDistance();
    }

    public double getRightSensorDistance() {
        return rightSensor.getDistance();
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

        STOW,
        IDLE,
        UP,
        DOWN
        // flags

    }

}
