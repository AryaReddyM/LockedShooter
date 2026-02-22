// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim motorSim;

  private boolean motorClosedLoop = false;
  private PIDController motorController = new PIDController(FlywheelConstants.kFlywheelP, 0, FlywheelConstants.kFlywheelD);
  private double motorFFVolts = 0.0;
  private double motorAppliedVolts = 0.0;

  public FlywheelIOSim() {
    // Create drive and turn sim models
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1),
            DCMotor.getNeo550(1));

    // Enable wrapping for turn PID
    motorController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
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
  public void setFlywheelVoltage(double volts) {
    motorClosedLoop = false;
    motorAppliedVolts = volts;
  }

  @Override
  public void setFlywheelSpeed(double pos) {
    motorClosedLoop = true;
    motorFFVolts = 0 * Math.signum(pos) + 0.0789 * pos;
    motorController.setSetpoint(pos);
  }

  @Override
  public void stopFlywheel() {
    setFlywheelSpeed(0);
  }

  @Override
  public boolean isAtSpeed(double a, double b) {
    return true;
  }
}