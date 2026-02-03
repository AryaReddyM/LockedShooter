package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax topMotor;
  private final SparkMax bottomMotor;

  public IntakeIOSpark(int topID, int bottomID) {
    topMotor = new SparkMax(topID, MotorType.kBrushless);
    bottomMotor = new SparkMax(bottomID, MotorType.kBrushless);;

    SparkMaxConfig topConfig = new SparkMaxConfig();
    topConfig.smartCurrentLimit(15);

    SparkMaxConfig bottomConfig = new SparkMaxConfig();
    bottomConfig.smartCurrentLimit(15);

    bottomConfig.follow(topMotor, true);

    topMotor.configure(topConfig, 
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomMotor.configure(bottomConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
      inputs.velocity = topMotor.getEncoder().getVelocity();
      inputs.voltage = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
  }

  @Override
  public void setVoltage(double volts) {
      topMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
      topMotor.stopMotor();
  }
}