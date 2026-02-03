package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;
import frc.robot.RobotState;

public class Intake extends StateMachine<Intake.State> implements IntakeIO {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final RobotState state;
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO, RobotState state) {
      super("Intake", State.UNDETERMINED, State.class);
      this.intakeIO = intakeIO;
      this.state = state;
      registerStateTransitions();
      registerStateCommands();
      enable();
  }

  public void run(double speed) {
    io.setVoltage(speed * 12.0);
  }

  public void stow() {
      intakeIO.setExtensionPosition(IntakeConstants.kExtensionStowSetpoint);
      intakeIO.stopRollers();
  }

  public void intakeidle() {
      intakeIO.setExtensionPosition(IntakeConstants.kExtensionIntakeSetpoint);
      intakeIO.stopRollers();
  }
  
  public void intake() {
      intakeIO.setExtensionPosition(IntakeConstants.kExtensionIntakeSetpoint);
      intakeIO.setRollerVoltage(IntakeConstants.kRollerIntakeVoltage);
  }

  public void outake(){
      intakeIO.setExtensionPosition(IntakeConstants.kExtensionOuttakeSetpoint);
      intakeIO.setRollerVoltage(IntakeConstants.kRollerOuttakeVoltage);
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
