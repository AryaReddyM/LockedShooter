// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.hood;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class HoodIOSim implements HoodIO {
  private final DCMotorSim motorSim;

  private boolean motorClosedLoop = false;
  private PIDController motorController = new PIDController(HoodConstants.kHoodP, 0, HoodConstants.kHoodD);
  private double motorFFVolts = 0.0;
  private double motorAppliedVolts = 0.0;

  public HoodIOSim() {
    // Create drive and turn sim models
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1),
            DCMotor.getNeo550(1));

    // Enable wrapping for turn PID
    motorController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Run closed-loop control
    if (motorClosedLoop) {
      motorAppliedVolts =
          motorFFVolts + motorController.calculate(motorSim.getAngularVelocityRadPerSec());
    } else {
      motorController.reset();
    }

    // Update simulation state
    motorSim.setInputVoltage(MathUtil.clamp(motorAppliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    // Update drive inputs
    inputs.posRad = motorSim.getAngularPositionRad();
    inputs.velPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = motorAppliedVolts;
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

  }

  @Override
  public void setHoodVoltage(double volts) {
    motorClosedLoop = false;
    motorAppliedVolts = volts;
  }

  @Override
  public void setHoodPosition(double pos, double ff) {
    motorClosedLoop = true;
    motorController.setSetpoint(pos);
  }

  @Override
  public double getHoodPosition() {
    return motorController.getSetpoint();
  }

  @Override
  public void stopHood() {
    setHoodPosition(0, 0);
  }
}