package frc.robot.subsystems.base;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class MotorIOSim implements MotorIO {
  private interface SimModel {
    void setInputVoltage(double volts);

    void update(double dtSeconds);

    double getPositionRad();

    double getVelocityRadPerSec();

    double getCurrentDrawAmps();

    void setPosition(double positionRad);
  }

  private enum ControlMode {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  private final SimModel sim;
  private final PIDController controller;
  private final double kS;
  private final double kV;
  private final boolean velocityControl;

  private ControlMode mode = ControlMode.OPEN_LOOP;
  private double setpoint = 0.0;
  private double runtimeFeedforward = 0.0;
  private double appliedVolts = 0.0;

  private MotorIOSim(
      SimModel sim, PIDController controller, double kS, double kV, boolean velocityControl) {
    this.sim = sim;
    this.controller = controller;
    this.kS = kS;
    this.kV = kV;
    this.velocityControl = velocityControl;
  }

  public static MotorIOSim flywheel(
      DCMotor motor, double moiKgM2, double gearRatio, double kP, double kS, double kV) {
    FlywheelSim fSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, moiKgM2, gearRatio), motor);
    SimModel model =
        new SimModel() {
          public void setInputVoltage(double volts) {
            fSim.setInputVoltage(volts);
          }

          public void update(double dt) {
            fSim.update(dt);
          }

          public double getPositionRad() {
            return 0.0; 
          }

          public double getVelocityRadPerSec() {
            return fSim.getAngularVelocityRadPerSec();
          }

          public double getCurrentDrawAmps() {
            return fSim.getCurrentDrawAmps();
          }

          public void setPosition(double positionRad) {
          }
        };
    return new MotorIOSim(model, new PIDController(kP, 0, 0), kS, kV, true);
  }

  public static MotorIOSim arm(
      DCMotor motor,
      double gearRatio,
      double armLengthMeters,
      double armMassKg,
      double minAngleRad,
      double maxAngleRad,
      boolean gravity,
      double startAngleRad,
      double kP) {
    SingleJointedArmSim aSim =
        new SingleJointedArmSim(
            motor,
            gearRatio,
            SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg),
            armLengthMeters,
            minAngleRad,
            maxAngleRad,
            gravity,
            startAngleRad);
    SimModel model =
        new SimModel() {
          public void setInputVoltage(double volts) {
            aSim.setInputVoltage(volts);
          }

          public void update(double dt) {
            aSim.update(dt);
          }

          public double getPositionRad() {
            return aSim.getAngleRads();
          }

          public double getVelocityRadPerSec() {
            return aSim.getVelocityRadPerSec();
          }

          public double getCurrentDrawAmps() {
            return aSim.getCurrentDrawAmps();
          }

          public void setPosition(double positionRad) {
            aSim.setState(positionRad, 0.0);
          }
        };
    return new MotorIOSim(model, new PIDController(kP, 0, 0), 0.0, 0.0, false);
  }

  public static MotorIOSim elevator(
      DCMotor motor,
      double gearRatio,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double kP) {
    ElevatorSim eSim =
        new ElevatorSim(
            motor,
            gearRatio,
            carriageMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            true,
            minHeightMeters);
    SimModel model =
        new SimModel() {
          public void setInputVoltage(double volts) {
            eSim.setInputVoltage(volts);
          }

          public void update(double dt) {
            eSim.update(dt);
          }

          public double getPositionRad() {
            return eSim.getPositionMeters();
          }

          public double getVelocityRadPerSec() {
            return eSim.getVelocityMetersPerSecond();
          }

          public double getCurrentDrawAmps() {
            return eSim.getCurrentDrawAmps();
          }

          public void setPosition(double positionMeters) {
            eSim.setState(positionMeters, 0.0);
          }
        };
    return new MotorIOSim(model, new PIDController(kP, 0, 0), 0.0, 0.0, false);
  }


  @Override
  public void updateInputs(MotorIOInputs inputs) {
    if (mode == ControlMode.CLOSED_LOOP) {
      double measurement = velocityControl ? sim.getVelocityRadPerSec() : sim.getPositionRad();
      double configFeedforward = velocityControl ? (kS * Math.signum(setpoint) + kV * setpoint) : 0.0;
      appliedVolts = configFeedforward + runtimeFeedforward + controller.calculate(measurement, setpoint);
    } 
    else {
      controller.reset();
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRad = sim.getPositionRad();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    mode = ControlMode.OPEN_LOOP;
    appliedVolts = volts;
  }

  @Override
  public void setPosition(double positionRad) {
    setPosition(positionRad, 0.0);
  }

  @Override
  public void setPosition(double positionRad, double feedforwardVolts) {
    mode = ControlMode.CLOSED_LOOP;
    setpoint = positionRad;
    runtimeFeedforward = feedforwardVolts;
  }

  @Override
  public void setMotionMagicPosition(double positionRad) {
    setPosition(positionRad, 0.0);
  }

  @Override
  public void setMotionMagicPosition(double positionRad, double feedforwardVolts) {
    setPosition(positionRad, feedforwardVolts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    setVelocity(velocityRadPerSec, 0.0);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double feedforwardVolts) {
    mode = ControlMode.CLOSED_LOOP;
    setpoint = velocityRadPerSec;
    runtimeFeedforward = feedforwardVolts;
  }

  @Override
  public void setMotionMagicVelocity(double velocityRadPerSec) {
    setVelocity(velocityRadPerSec, 0.0);
  }

  @Override
  public void setMotionMagicVelocity(double velocityRadPerSec, double feedforwardVolts) {
    setVelocity(velocityRadPerSec, feedforwardVolts);
  }

  @Override
  public void setDutyCycle(double fraction) {
    setVoltage(fraction * 12.0);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setEncoderPosition(double positionRad) {
    sim.setPosition(positionRad);
  }
}
