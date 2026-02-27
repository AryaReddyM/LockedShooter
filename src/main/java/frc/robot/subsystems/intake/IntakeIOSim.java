// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim motorSim;
  private final DCMotorSim extensionSim;

  private boolean motorClosedLoop = false;
  private PIDController motorController = new PIDController(IntakeConstants.kRollerSimP, 0, IntakeConstants.kRollerSimD);
  private double motorFFVolts = 0.0;
  private double motorAppliedVolts = 0.0;

  private boolean extensionClosedLoop = false;
  private PIDController extensionController = new PIDController(IntakeConstants.kExtensionSimP, 0, IntakeConstants.kExtensionSimD);
  private double extensionFFVolts = 0.0;
  private double extensionAppliedVolts = 0.0;
  private double desiredPos = 0.0;

  public IntakeIOSim() {
    // Create drive and turn sim models
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1),
            DCMotor.getNeo550(1));

    extensionSim =
      new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1),
            DCMotor.getNeo550(1));
    // Enable wrapping for turn PID
    motorController.enableContinuousInput(-Math.PI, Math.PI);
    extensionController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Run closed-loop control
    if (motorClosedLoop) {
      motorAppliedVolts =
          motorFFVolts + motorController.calculate(motorSim.getAngularVelocityRadPerSec());
    } else {
      motorController.reset();
    }

    if (extensionClosedLoop) {
      extensionAppliedVolts = extensionController.calculate(extensionSim.getAngularPositionRad());
    } else {
      extensionController.reset();
    }

    // Update simulation state
    motorSim.setInputVoltage(MathUtil.clamp(motorAppliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    extensionSim.setInputVoltage(MathUtil.clamp(extensionAppliedVolts, -12.0, 12.0));
    extensionSim.update(0.02);

    // Update drive inputs
    inputs.rollerPosRad = motorSim.getAngularPositionRad();
    inputs.rollerVelPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = motorAppliedVolts;
    inputs.rollerCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());

    inputs.extensionPosRad = extensionSim.getAngularPositionRad();
    inputs.extensionVelPerSec = extensionSim.getAngularVelocityRadPerSec();
    inputs.extensionAppliedVolts = extensionAppliedVolts;
    inputs.extensionCurrentAmps = Math.abs(extensionSim.getCurrentDrawAmps());

    inputs.desiredExtensionPos = desiredPos;
  }

  @Override
  public void setRollerVoltage(double volts) {
    motorClosedLoop = false;
    motorAppliedVolts = volts;
  }

  @Override
  public void setRollerSpeed(double pos) {
    motorClosedLoop = true;
    motorFFVolts = 0 * Math.signum(pos) + 0.0789 * pos;
    motorController.setSetpoint(pos);
  }

  @Override
  public void stopRollers() {
    setRollerSpeed(0);
  }

  @Override
  public void setExtensionPosition(double pos) {
    pos *= -7.0/500.0;
    this.desiredPos = pos;
    extensionClosedLoop = true;
    extensionController.setSetpoint(pos);
  }

   @Override
  public void stopExtension() {
    setExtensionPosition(0);
  }
}