// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class ClimbIOSim implements ClimbIO {
  private final DCMotorSim motorSim;

  private boolean motorClosedLoop = false;
  private PIDController motorController = new PIDController(ClimbConstants.climbKp, 0, ClimbConstants.climbKd);
  private double motorFFVolts = 0.0;
  private double motorAppliedVolts = 0.0;

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
      motorAppliedVolts =
          motorFFVolts + motorController.calculate(motorSim.getAngularVelocityRadPerSec());
    } else {
      motorController.reset();
    }

    // Update simulation state
    motorSim.setInputVoltage(MathUtil.clamp(motorAppliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    // Update drive inputs
    inputs.climbPositionRad = motorSim.getAngularPositionRad();
    inputs.climbVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.climbAppliedVolts = motorAppliedVolts;
    inputs.climbCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());

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
  }

  @Override
  public void stopClimb() {
    setClimbPosition(0);
  }
}