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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.ServoMotorSubsystem;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.logging.GetTuned;
import frc.robot.util.field.TrenchZone;

public class Intake extends ServoMotorSubsystem {

    private final RobotState state;

    // Two independent motors: the deploy arm (position) and the rollers (velocity).
    private final MotorIO rollersIo;
    private final MotorIOInputsAutoLogged rollerInputs = new MotorIOInputsAutoLogged();

    private final LoggedMechanism2d intakeMechanism = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismLigament2d intakeLigament;

    private double desiredExtensionPos = 0.0;
    private Consumer<Object> override;
    private State stateValue = State.IDLE;

    public Intake(MotorIO extensionIo, MotorIO rollersIo, RobotState state) {
        super(extensionIo, "Intake/Extension", IntakeConstants.kExtensionDeviationErr);
        this.rollersIo = rollersIo;
        this.state = state;

        LoggedMechanismRoot2d root = intakeMechanism.getRoot("intake", 1.85, 0);
        LoggedMechanismLigament2d holder = root.append(new LoggedMechanismLigament2d("core", 0.05, 90));
        intakeLigament = holder.append(new LoggedMechanismLigament2d("bar", 0.3, 0, 10, new Color8Bit(255, 0, 0)));
    }

    @Override
    public void periodic() {
        super.periodic();

        rollersIo.updateInputs(rollerInputs);
        Logger.processInputs("Intake/Rollers", rollerInputs);

        Logger.recordOutput("Intake/Mechanism", intakeMechanism);

        Logger.recordOutput("Intake/Pose",
                new Pose3d()
                        .plus(IntakeConstants.intakeOrigin)
                        .plus(new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0, -(desiredExtensionPos + 90), 0))));
        Logger.recordOutput("Intake/ExtensionPose",
                new Pose3d()
                        .plus(new Transform3d()).plus(new Transform3d(new Translation3d(
                                (stateValue != State.STOW) ? Units.inchesToMeters(11) : Units.inchesToMeters(0), 0,
                                Units.inchesToMeters(-2.7)), new Rotation3d())));

        if (TrenchZone.intakeLowerRequired(state)) {
            setExtension(GetTuned.getNumber("Intake/Extension Intake Setpoint", IntakeConstants.kExtensionIntakeSetpoint));
        } else if (override != null) {
            override.accept(null);
        } else if (stateValue == State.INTAKE) {
            intake();
        } else if (stateValue == State.OUTAKE) {
            outake();
        } else if (stateValue == State.STOW) {
            stow();
        } else if (stateValue == State.IDLE) {
            intakeidle();
        } else if (stateValue == State.CLIMB_TOW) {
            climbTow();
        } else if (stateValue == State.SHAKE) {
            shake();
        } else if (stateValue == State.STOP) {
            stop();
        }
        Logger.recordOutput("Intake/Overriden", override != null);
        Logger.recordOutput("Intake/State", stateValue.toString());
    }

    private void setExtension(double position) {
        desiredExtensionPos = position;
        applySetpoint(Setpoint.motionMagicPosition(position));
    }

    public void stow() {
        setExtension(GetTuned.getNumber("Intake/Extension Stow Setpoint", IntakeConstants.kExtensionStowSetpoint));
        rollersIo.setVelocity(0);
    }

    public void intakeidle() {
        setExtension(GetTuned.getNumber("Intake/Extension Intake Setpoint", IntakeConstants.kExtensionIntakeSetpoint));
        rollersIo.setVelocity(0);
    }

    public void intake() {
        setExtension(GetTuned.getNumber("Intake/Extension Intake Setpoint", IntakeConstants.kExtensionIntakeSetpoint));
        rollersIo.setVelocity(GetTuned.getNumber("Intake/Roller Intake Speed", IntakeConstants.kRollerIntakeSpeed));
    }

    public void outake() {
        setExtension(GetTuned.getNumber("Intake/Extension Outtake Setpoint", IntakeConstants.kExtensionOuttakeSetpoint));
        rollersIo.setVelocity(GetTuned.getNumber("Intake/Roller Outtake Speed", IntakeConstants.kRollerOuttakeSpeed));
    }

    public void climbTow() {
        setExtension(GetTuned.getNumber("Intake/Extension Tow Setpoint", IntakeConstants.kExtensionClimbTowSetpoint));
        rollersIo.setVelocity(0);
    }

    public void shake() {
        setExtension(GetTuned.getNumber("Intake/Extension Shake Setpoint", IntakeConstants.kExtensionShakeSetpoint));
        rollersIo.setVelocity(GetTuned.getNumber("Intake/Roller Intake Speed", IntakeConstants.kRollerIntakeSpeed));
    }

    public void intakeRoll() {
        rollersIo.setVelocity(GetTuned.getNumber("Intake/Roller Intake Speed", IntakeConstants.kRollerIntakeSpeed));
    }

    public void outakeRoll() {
        rollersIo.setVelocity(GetTuned.getNumber("Intake/Roller Outtake Speed", IntakeConstants.kRollerOuttakeSpeed));
    }

    public void stopRoll() {
        rollersIo.setVelocity(0);
    }

    public void stop() {
        super.stop();
        rollersIo.setVelocity(0);
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

        STOP,
        STOW,
        IDLE,
        INTAKE,
        OUTAKE,
        CLIMB_TOW,
        SHAKE

        // flags

    }
}
