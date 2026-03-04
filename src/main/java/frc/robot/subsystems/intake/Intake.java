package frc.robot.subsystems.intake;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.util.GetTuned;
import frc.robot.util.state.StateMachine;

public class Intake extends StateMachine<Intake.State> implements IntakeIO {

    private final RobotState state;
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final LoggedMechanism2d intakeMechanism = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismLigament2d intakeLigament;

    private Consumer<Object> override;

    public Intake(IntakeIO intakeIO, RobotState state) {
        super("Intake", State.STOW, State.class);
        this.intakeIO = intakeIO;
        this.state = state;

        LoggedMechanismRoot2d root = intakeMechanism.getRoot("intake", 1.85, 0);
        LoggedMechanismLigament2d holder = root.append(new LoggedMechanismLigament2d("core", 0.05, 90));
        intakeLigament = holder.append(new LoggedMechanismLigament2d("bar", 0.3, 0, 10, new Color8Bit(255, 0, 0)));

        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        intakeLigament.setAngle(Rotation2d.fromRadians(inputs.extensionPosRad));
        Logger.recordOutput("Intake/Mechanism", intakeMechanism);

        Logger.recordOutput("Intake/Pose",
                new Pose3d()
                        .plus(IntakeConstants.intakeOrigin)
                        .plus(new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0, -inputs.desiredExtensionPos, 0))));
        Logger.recordOutput("Intake/ExtensionPose",
                new Pose3d()
                        .plus(new Transform3d()).plus(new Transform3d(new Translation3d(
                            (getState() != State.STOW) ? Units.inchesToMeters(11) : Units.inchesToMeters(0) ,0,Units.inchesToMeters(-2.7)
                        ), new Rotation3d())));

        if (override != null) {
            override.accept(null);
        }else if (getState() == State.INTAKE) {
            intake();
        } else if (getState() == State.OUTAKE) {
            outake();
        } else if (getState() == State.STOW) {
            stow();
        } else if (getState() == State.IDLE) {
            intakeidle();
        } else if (getState() == State.STOP) {
            stop();
        }
    }

    public void stow() {
        intakeIO.setExtensionPosition(
                GetTuned.getNumber("Intake/Extension Stow Setpoint", IntakeConstants.kExtensionStowSetpoint));
        intakeIO.stopRollers();
    }

    public void intakeidle() {
        intakeIO.setExtensionPosition(
                GetTuned.getNumber("Intake/Extension Intake Setpoint", IntakeConstants.kExtensionIntakeSetpoint));
        intakeIO.stopRollers();
    }

    public void intake() {
        intakeIO.setExtensionPosition(
                GetTuned.getNumber("Intake/Extension Intake Setpoint", IntakeConstants.kExtensionIntakeSetpoint));
        intakeIO.setRollerSpeed(GetTuned.getNumber("Intake/Roller Intake Speed", IntakeConstants.kRollerIntakeSpeed));
    }

    public void outake() {
        intakeIO.setExtensionPosition(
                GetTuned.getNumber("Intake/Extension Outtake Setpoint", IntakeConstants.kExtensionOuttakeSetpoint));
        intakeIO.setRollerSpeed(GetTuned.getNumber("Intake/Roller Outtake Speed", IntakeConstants.kRollerOuttakeSpeed));
    }

    public void intakeRoll() {
        intakeIO.setRollerSpeed(GetTuned.getNumber("Intake/Roller Intake Speed", IntakeConstants.kRollerIntakeSpeed));
    }

    public void outakeRoll() {
        intakeIO.setRollerSpeed(GetTuned.getNumber("Intake/Roller Outtake Speed", IntakeConstants.kRollerOuttakeSpeed));
    }

    public void stopRoll() {
        intakeIO.stopRollers();
    }

    public void stop() {
        intakeIO.stopExtension();
        intakeIO.stopRollers();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.STOW, State.IDLE, State.INTAKE, State.OUTAKE);
    }

    private void registerStateCommands() {

    }

    @Override
    protected void determineSelf() {
        setState(State.STOW);
    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
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
