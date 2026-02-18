package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.GetTuned;
import frc.robot.util.state.StateMachine;

public class Climb extends StateMachine<Climb.State> implements ClimbIO {

    private final RobotState state;
    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final LoggedMechanism2d climbMechanism = new LoggedMechanism2d(3, 4);

    private final LoggedMechanismLigament2d climbElevatorExtension;

    public Climb(ClimbIO climbIO, RobotState state) {
        super("Climb", State.UNDETERMINED, State.class);
        this.climbIO = climbIO;
        this.state = state;

        LoggedMechanismRoot2d climbRoot = climbMechanism.getRoot("Climber", 1.85, 0);
        LoggedMechanismLigament2d climbElevatorBase = climbRoot
                .append(new LoggedMechanismLigament2d("elevator", ClimbConstants.kClimberBaseHeight, 90));
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
        Logger.processInputs("Climb", inputs);

        climbElevatorExtension.setLength(inputs.desiredPos);
        Logger.recordOutput("Climb/Mechanism", climbMechanism);
    }

    public void stow() {
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Stow Setpoint", ClimbConstants.kClimbStowPos));
    }

    public void up() {
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Up Setpoint", ClimbConstants.kClimbUpPos));

    }

    public void down() {
        climbIO.setClimbPosition(GetTuned.getNumber("Climb/Down Setpoint", ClimbConstants.kClimbDownPos));
    }

    public void stop() {
        climbIO.stopClimb();
    }

    public Command zero() {
        return run(() -> {
            climbIO.setCurrentLimit(ClimbConstants.kLowerCurrentLimit); // lower current limit
            climbIO.setMotorOutput(ClimbConstants.kLowerMotorOutput); // slow downward
        })
                .until(() -> climbIO.getMotorCurrent() > ClimbConstants.kZeroCurrentThreshold) // detect stall
                .finallyDo(interrupted -> {
                    climbIO.setMotorOutput(0);
                    climbIO.zeroEncoder();
                    climbIO.setCurrentLimit(ClimbConstants.kClimbCurrentLimit);
                });

    }

    private void registerStateTransitions() {
        addOmniTransitions(State.STOW, State.IDLE, State.UP, State.CLIMB);
    }

    private void registerStateCommands() {
        registerStateCommand(State.STOW, Commands.run(() -> stow()));
        registerStateCommand(State.IDLE, Commands.run(() -> stop()));
        registerStateCommand(State.UP, Commands.run(() -> up()));
        registerStateCommand(State.CLIMB, Commands.run(() -> down()));
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
        CLIMB,
        // flags

    }

}
