// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.turret;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class TurretIOSim implements TurretIO {
  private final DCMotorSim motorSim;

  private boolean motorClosedLoop = false;
  private PIDController motorController = new PIDController(TurretConstants.kTurretSimP, 0, TurretConstants.kTurretSimD);
  private double motorFFVolts = 0.0;
  private double motorAppliedVolts = 0.0;
  private double desiredPos = 0.0;

  public TurretIOSim() {
    // Create drive and turn sim models
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1),
            DCMotor.getNeo550(1));

    // Enable wrapping for turn PID
    motorController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
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
    inputs.turretPos = motorSim.getAngularPositionRad();
    inputs.turretVelRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = motorAppliedVolts;
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

    inputs.turretRotation2d = new Rotation2d(inputs.turretPos);
    inputs.desiredPos = this.desiredPos;

  }

  @Override
  public void setTurretVoltage(double volts) {
    motorClosedLoop = false;
    motorAppliedVolts = volts;
  }

  @Override
  public void setTurretPosition(double pos, double ff) {
    motorClosedLoop = true;
    motorController.setSetpoint(pos);
    this.desiredPos = pos;
  }

  @Override
  public double getTurretPosition() {
    return motorController.getSetpoint();
  }

  // no vel and turret to rot im tired

  @Override
  public void stopTurret() {
    setTurretPosition(0, 0);
  }
}