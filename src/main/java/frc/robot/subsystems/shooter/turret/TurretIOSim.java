package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {

  private final DCMotorSim motorSim;

  private boolean closedLoop = false;
  private final PIDController controller =
      new PIDController(TurretConstants.kTurretSimP, 0, TurretConstants.kTurretSimD);

  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;
  private double desiredPos = 0.0;

  public TurretIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1),
                0.001,
                1.0),
            DCMotor.getNeo550(1));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    double position = motorSim.getAngularPositionRad();

    // Soft limits (match Spark)
    if (position > TurretConstants.kForwardSoftLimit) {
      motorSim.setState(TurretConstants.kForwardSoftLimit, 0);
      position = TurretConstants.kForwardSoftLimit;
    } else if (position < TurretConstants.kBackwardSoftLimit) {
      motorSim.setState(TurretConstants.kBackwardSoftLimit, 0);
      position = TurretConstants.kBackwardSoftLimit;
    }

    // Closed loop control
    if (closedLoop) {
      double pidOutput = controller.calculate(position);
      appliedVolts = pidOutput + ffVolts;
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    // Update inputs (match Spark structure)
    inputs.turretRotation2d =
        new Rotation2d(motorSim.getAngularPositionRad());

    inputs.turretVelRadPerSec =
        motorSim.getAngularVelocityRadPerSec();

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    inputs.desiredPos = desiredPos;
  }

  @Override
  public void setTurretVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setTurretPosition(double position, double ff) {
    closedLoop = true;

    // Clamp to soft limits like real controller
    position =
        MathUtil.clamp(
            position,
            TurretConstants.kBackwardSoftLimit,
            TurretConstants.kForwardSoftLimit);

    controller.setSetpoint(position);
    ffVolts = ff;
    desiredPos = position;
  }

  @Override
  public double getTurretPosition() {
    return controller.getSetpoint();
  }

  @Override
  public double getTurretVelocity() {
    return motorSim.getAngularVelocityRadPerSec();
  }

  @Override
  public Rotation2d getRobotToTurretRotation() {
    return new Rotation2d(motorSim.getAngularPositionRad());
  }

  @Override
  public void stopTurret() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}