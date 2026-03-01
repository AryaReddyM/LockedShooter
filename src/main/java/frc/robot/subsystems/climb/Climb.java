package frc.robot.subsystems.climb;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.commands.ActionCommands;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.GetTuned;
import frc.robot.util.state.StateMachine;

public class Climb extends StateMachine<Climb.State> implements ClimbIO {

    private final RobotState state;
    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final BeamBreakerInputsAutoLogged leftInputs = new BeamBreakerInputsAutoLogged();
    private final BeamBreakerInputsAutoLogged rightInputs = new BeamBreakerInputsAutoLogged();
    private final LoggedMechanism2d climbMechanism = new LoggedMechanism2d(3, 4);

    private final BeamBreakerIO leftSensor;
    private final BeamBreakerIO rightSensor;

    private final LoggedMechanismLigament2d climbElevatorExtension;
    private Consumer<Object> override;

    public Climb(ClimbIO climbIO, BeamBreakerIO leftSensor, BeamBreakerIO rightSensor, RobotState state) {
        super("Climb", State.UNDETERMINED, State.class);
        this.climbIO = climbIO;
        this.state = state;
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;

        LoggedMechanismRoot2d climbRoot = climbMechanism.getRoot("Climber", 1.85, 0);
        LoggedMechanismLigament2d climbElevatorBase = climbRoot
                .append(new LoggedMechanismLigament2d("elevator", ClimbConstants.kClimberBaseHeight, 90)); // TODO
                                                                                                           // conversion
                                                                                                           // because
                                                                                                           // its not in
                                                                                                           // meters!
        climbElevatorExtension = climbElevatorBase
                .append(new LoggedMechanismLigament2d("extension", 0, 0, 10, new Color8Bit(255, 0, 0)));

        registerStateTransitions();
        registerStateCommands();
        enable();

        SmartDashboard.putData("Climb Zero", zero().withName("Climb Zero"));
    }

    @Override
    public void update() {
        climbIO.updateInputs(inputs);
        leftSensor.updateInputs(leftInputs);
        rightSensor.updateInputs(rightInputs);

        Logger.processInputs("Climb", inputs);
        Logger.processInputs("Left Sensor", leftInputs);
        Logger.processInputs("Right Sensor", rightInputs);

        climbElevatorExtension.setLength(inputs.desiredPos);
        Logger.recordOutput("Climb/Mechanism", climbMechanism);

        Logger.recordOutput("Climb/Pose",
                new Pose3d()
                        .plus(new Transform3d()).plus(
                                new Transform3d(
                                        new Translation3d(0, 0,
                                                inputs.desiredPos * ClimbConstants.kClimbPositionConversionFactor),
                                        new Rotation3d())));

        if (override!= null) {
            override.accept(null);   
        }
        else if (getState() == State.STOW) {
            stow();
        } else if (getState() == State.IDLE) {
            stop();
        } else if (getState() == State.UP) {
            up();
        } else if (getState() == State.DOWN) {
            down();
        }
        Logger.recordOutput("Climb/Overriden", override!=null);
    }

    public void stow() {
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Stow Setpoint", ClimbConstants.kClimbStowPos));
    }

    public void up() {
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Up Setpoint", ClimbConstants.kClimbUpPos));

    }

    public void down() {
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Down Setpoint", ClimbConstants.kClimbDownPos),
                ClosedLoopSlot.kSlot1);
    }

    public void stop() {
        climbIO.stopClimb();
    }

    public Command zero() {
        return run(() -> {
            climbIO.setMotorOutput(GetTuned.getNumber("Climb/Lower Motor Output", ClimbConstants.kLowerMotorOutput));
        })
                .beforeStarting(() -> {
                    climbIO.setCurrentLimit(
                            GetTuned.getNumber("Climb/Lower Current Limit", ClimbConstants.kLowerCurrentLimit));
                })
                .until(() -> inputs.currentAmps > GetTuned.getNumber("Climb/Zero Current Threshold",
                        ClimbConstants.kZeroCurrentThreshold))
                .finallyDo(interrupted -> {
                    climbIO.stopClimb();
                    climbIO.zeroEncoder();
                    climbIO.setCurrentLimit(ClimbConstants.kClimbCurrentLimit);
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

    private void registerStateTransitions() {
        addOmniTransitions(State.STOW, State.IDLE, State.UP, State.DOWN);
    }

    private void registerStateCommands() {

    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public enum State {
        UNDETERMINED,

        STOW,
        IDLE,
        UP,
        DOWN,
        // flags

    }

}
