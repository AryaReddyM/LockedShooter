package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.GetTuned;
import frc.robot.util.state.StateMachine;

public class Intake extends StateMachine<Intake.State> implements IntakeIO{

    private final RobotState state;
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final LoggedMechanism2d intakeMechanism = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismLigament2d intakeLigament;

    public Intake(IntakeIO intakeIO, RobotState state) {
        super("Intake", State.UNDETERMINED, State.class);
        this.intakeIO = intakeIO;
        this.state = state;

        LoggedMechanismRoot2d root = intakeMechanism.getRoot("intake", 1.85, 0);
        LoggedMechanismLigament2d holder = root.append(new LoggedMechanismLigament2d("core", 0.05, 90));
        intakeLigament = holder.append(new LoggedMechanismLigament2d("bar", 0.3,0, 10, new Color8Bit(255, 0, 0)));

        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        intakeLigament.setAngle(Rotation2d.fromRadians(inputs.desiredExtensionPos));
        Logger.recordOutput("Intake/Mechanism", intakeMechanism);
    }

    public void stow() {
        intakeIO.setExtensionPosition(GetTuned.getNumber("Intake/Extension Stow Setpoint", IntakeConstants.kExtensionStowSetpoint));
        intakeIO.stopRollers();
    }

    public void intakeidle() {
        intakeIO.setExtensionPosition(GetTuned.getNumber("Intake/Extension Intake Setpoint", IntakeConstants.kExtensionIntakeSetpoint));
        intakeIO.stopRollers();
    }
    
    public void intake() {
        intakeIO.setExtensionPosition(GetTuned.getNumber("Intake/Extension Intake Setpoint", IntakeConstants.kExtensionIntakeSetpoint));
        intakeIO.setRollerSpeed(GetTuned.getNumber("Intake/Roller Intake Speed", IntakeConstants.kRollerIntakeSpeed));
    }

    public void outake(){
        intakeIO.setExtensionPosition(GetTuned.getNumber("Intake/Extension Outtake Setpoint", IntakeConstants.kExtensionOuttakeSetpoint));
        intakeIO.setRollerSpeed(GetTuned.getNumber("Intake/Roller Outtake Speed", IntakeConstants.kRollerOuttakeSpeed));
    }

    public void stop() {
        intakeIO.stopExtension();
        intakeIO.stopRollers();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.STOW, State.IDLE, State.INTAKE, State.OUTAKE);
    }

    private void registerStateCommands() {
        registerStateCommand(State.STOP, Commands.run(() -> stop()));
        registerStateCommand(State.STOW, Commands.run(() -> stow()));
        registerStateCommand(State.IDLE, Commands.run(() -> intakeidle()));
        registerStateCommand(State.INTAKE, Commands.run(() -> intake()));
        registerStateCommand(State.OUTAKE, Commands.run(() -> outake()));
    }

     @Override
    protected void determineSelf() {
        setState(State.STOP);
    }
    
    public enum State {
        UNDETERMINED,

        STOP,
        STOW,
        IDLE,
        INTAKE,
        OUTAKE

        // flags

    }
}

