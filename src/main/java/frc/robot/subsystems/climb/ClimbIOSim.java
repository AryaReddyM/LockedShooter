// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;


import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class ClimbIOSim implements ClimbIO {
  private final DCMotorSim motorSim;

  private boolean motorClosedLoop = false;
  private PIDController motorController = new PIDController(ClimbConstants.kClimbSimP, 0, ClimbConstants.kClimbSimD);
  private double motorFFVolts = 0.0;
  private double motorAppliedVolts = 0.0;
  private double desiredPos = 0.0;

  public ClimbIOSim() {
    // Create drive and turn sim models
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1),
            DCMotor.getNeo550(1));

    // Enable wrapping for turn PID
    motorController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Run closed-loop control
    if (motorClosedLoop) {
      motorAppliedVolts = motorController.calculate(motorSim.getAngularPositionRad());
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

    inputs.desiredPos = this.desiredPos;

  }

  @Override
  public void setClimbVoltage(double volts) {
    motorClosedLoop = false;
    motorAppliedVolts = volts;
  }

  @Override
  public void setClimbPosition(double pos) {
    motorClosedLoop = true;
    motorController.setSetpoint(pos);
    this.desiredPos = pos;
  }

  @Override
  public void setClimbPosition(double pos, ClosedLoopSlot slot) {
    setClimbPosition(pos);
  }

  // no vel and turret to rot im tired

  @Override
  public void stopClimb() {
    setClimbPosition(0);
  }
}