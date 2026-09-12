package frc.robot.subsystems.base;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Physics sim implementation of {@link MotorIO}.
 *
 * <p>The rest of the codebase treats a {@code MotorIO} as speaking a mechanism's <i>native</i>
 * units — whatever the real controller was configured to report. {@link MotorIOTalonFX} gets those
 * from {@code SensorToMechanismRatio} and {@link MotorIOSpark} from its conversion factors, so for
 * example the flywheel reports surface meters and the intake extension reports degrees, both under
 * the historical {@code positionRad} field name.
 *
 * <p>This class simulates in SI mechanism units (rad / rad per sec, or meters for an elevator) and
 * converts on the way in and out with {@code unitsPerSi}, so sim reports the same numbers the real
 * robot does. Velocity feedforward is derived from the motor curve rather than hand tuned, which
 * leaves near zero steady state error and lets {@code kP} deal only with load.
 */
public class MotorIOSim implements MotorIO {
  private static final double kNominalVoltage = 12.0;
  private static final double kAmbientTempCelsius = 25.0;
  private static final double kTempRisePerAmp = 0.35;
  private static final double kLoopPeriodSecs = 0.02;

  private interface SimModel {
    void setInputVoltage(double volts);

    void update(double dtSeconds);

    /** Position in SI mechanism units (radians, or meters for an elevator). */
    double getPositionSi();

    /** Velocity in SI mechanism units per second. */
    double getVelocitySi();

    double getCurrentDrawAmps();

    void setPositionSi(double positionSi);
  }

  private enum ControlMode {
    OPEN_LOOP,
    POSITION,
    VELOCITY
  }

  private final SimModel sim;
  private final PIDController controller;

  /** Native controller units per SI unit. 1.0 when the mechanism already reports rad or meters. */
  private final double unitsPerSi;

  /** Volts per SI unit of velocity needed to hold speed against back EMF. */
  private final double voltsPerSiVelocity;

  private ControlMode mode = ControlMode.OPEN_LOOP;

  /** Commanded position or velocity, in the mechanism's native units. */
  private double setpoint = 0.0;

  private double runtimeFeedforward = 0.0;
  private double appliedVolts = 0.0;
  private double currentLimitAmps = Double.POSITIVE_INFINITY;
  private boolean brakeMode = true;

  private MotorIOSim(
      SimModel sim, PIDController controller, double unitsPerSi, double voltsPerSiVelocity) {
    this.sim = sim;
    this.controller = controller;
    this.unitsPerSi = unitsPerSi;
    this.voltsPerSiVelocity = voltsPerSiVelocity;
  }

  /** Volts needed per rad/s at the mechanism output for a given motor and reduction. */
  private static double voltsPerRadPerSec(DCMotor motor, double gearRatio) {
    return kNominalVoltage / (motor.freeSpeedRadPerSec / gearRatio);
  }

  /**
   * A free spinning wheel (flywheel, feeder roller, kicker).
   *
   * @param unitsPerRad native units the controller reports per radian of mechanism rotation
   */
  public static MotorIOSim flywheel(
      DCMotor motor, double moiKgM2, double gearRatio, double unitsPerRad, double kP, double kD) {
    FlywheelSim fSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, moiKgM2, gearRatio), motor);
    SimModel model =
        new SimModel() {
          // FlywheelSim carries no position state, so integrate velocity to keep
          // positionRad meaningful (it used to be hard coded to zero).
          private double positionRad = 0.0;

          public void setInputVoltage(double volts) {
            fSim.setInputVoltage(volts);
          }

          public void update(double dt) {
            fSim.update(dt);
            positionRad += fSim.getAngularVelocityRadPerSec() * dt;
          }

          public double getPositionSi() {
            return positionRad;
          }

          public double getVelocitySi() {
            return fSim.getAngularVelocityRadPerSec();
          }

          public double getCurrentDrawAmps() {
            return fSim.getCurrentDrawAmps();
          }

          public void setPositionSi(double position) {
            positionRad = position;
          }
        };
    return new MotorIOSim(
        model, new PIDController(kP, 0, kD), unitsPerRad, voltsPerRadPerSec(motor, gearRatio));
  }

  /**
   * A rotating mechanism (turret, hood, intake deploy, climb winch).
   *
   * <p>Limits and the start angle are given in native units so callers can pass their existing soft
   * limits straight through.
   */
  public static MotorIOSim arm(
      DCMotor motor,
      double gearRatio,
      double armLengthMeters,
      double armMassKg,
      double minUnits,
      double maxUnits,
      boolean gravity,
      double startUnits,
      double unitsPerRad,
      double kP,
      double kD) {
    double minRad = Math.min(minUnits, maxUnits) / unitsPerRad;
    double maxRad = Math.max(minUnits, maxUnits) / unitsPerRad;
    SingleJointedArmSim aSim =
        new SingleJointedArmSim(
            motor,
            gearRatio,
            SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg),
            armLengthMeters,
            minRad,
            maxRad,
            gravity,
            MathUtil.clamp(startUnits / unitsPerRad, minRad, maxRad));
    SimModel model =
        new SimModel() {
          public void setInputVoltage(double volts) {
            aSim.setInputVoltage(volts);
          }

          public void update(double dt) {
            aSim.update(dt);
          }

          public double getPositionSi() {
            return aSim.getAngleRads();
          }

          public double getVelocitySi() {
            return aSim.getVelocityRadPerSec();
          }

          public double getCurrentDrawAmps() {
            return aSim.getCurrentDrawAmps();
          }

          public void setPositionSi(double positionRad) {
            aSim.setState(MathUtil.clamp(positionRad, minRad, maxRad), 0.0);
          }
        };
    return new MotorIOSim(
        model, new PIDController(kP, 0, kD), unitsPerRad, voltsPerRadPerSec(motor, gearRatio));
  }

  /** A linear elevator. Reports meters, which is already the native unit for this mechanism. */
  public static MotorIOSim elevator(
      DCMotor motor,
      double gearRatio,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double kP,
      double kD) {
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

          public double getPositionSi() {
            return eSim.getPositionMeters();
          }

          public double getVelocitySi() {
            return eSim.getVelocityMetersPerSecond();
          }

          public double getCurrentDrawAmps() {
            return eSim.getCurrentDrawAmps();
          }

          public void setPositionSi(double positionMeters) {
            eSim.setState(MathUtil.clamp(positionMeters, minHeightMeters, maxHeightMeters), 0.0);
          }
        };
    // A drum converts motor rotation into carriage travel, so free speed in meters
    // per second is free speed in rad/s times the drum radius.
    double voltsPerMeterPerSec =
        kNominalVoltage / (motor.freeSpeedRadPerSec / gearRatio * drumRadiusMeters);
    return new MotorIOSim(model, new PIDController(kP, 0, kD), 1.0, voltsPerMeterPerSec);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    // The controller runs in native units, so sim gains read as volts per native unit
    // (per radian, per degree, per surface m/s, ...) whatever the mechanism reports.
    switch (mode) {
      case POSITION ->
          appliedVolts =
              runtimeFeedforward + controller.calculate(sim.getPositionSi() * unitsPerSi, setpoint);
      case VELOCITY ->
          appliedVolts =
              (setpoint / unitsPerSi) * voltsPerSiVelocity
                  + runtimeFeedforward
                  + controller.calculate(sim.getVelocitySi() * unitsPerSi, setpoint);
      default -> controller.reset();
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -kNominalVoltage, kNominalVoltage);
    sim.setInputVoltage(appliedVolts);
    sim.update(kLoopPeriodSecs);

    double currentAmps = Math.min(Math.abs(sim.getCurrentDrawAmps()), currentLimitAmps);

    inputs.positionRad = sim.getPositionSi() * unitsPerSi;
    inputs.velocityRadPerSec = sim.getVelocitySi() * unitsPerSi;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    inputs.tempCelsius = kAmbientTempCelsius + currentAmps * kTempRisePerAmp;
  }

  @Override
  public void setVoltage(double volts) {
    mode = ControlMode.OPEN_LOOP;
    appliedVolts = volts;
  }

  @Override
  public void setPosition(double position) {
    setPosition(position, 0.0);
  }

  @Override
  public void setPosition(double position, double feedforwardVolts) {
    if (mode != ControlMode.POSITION) {
      controller.reset();
      mode = ControlMode.POSITION;
    }
    setpoint = position;
    runtimeFeedforward = feedforwardVolts;
  }

  @Override
  public void setMotionMagicPosition(double position) {
    setPosition(position, 0.0);
  }

  @Override
  public void setMotionMagicPosition(double position, double feedforwardVolts) {
    setPosition(position, feedforwardVolts);
  }

  @Override
  public void setMotionMagicPosition(double position, int slot) {
    setPosition(position, 0.0);
  }

  @Override
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0.0);
  }

  @Override
  public void setVelocity(double velocity, double feedforwardVolts) {
    if (mode != ControlMode.VELOCITY) {
      controller.reset();
      mode = ControlMode.VELOCITY;
    }
    setpoint = velocity;
    runtimeFeedforward = feedforwardVolts;
  }

  @Override
  public void setMotionMagicVelocity(double velocity) {
    setVelocity(velocity, 0.0);
  }

  @Override
  public void setMotionMagicVelocity(double velocity, double feedforwardVolts) {
    setVelocity(velocity, feedforwardVolts);
  }

  @Override
  public void setDutyCycle(double fraction) {
    setVoltage(fraction * kNominalVoltage);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setEncoderPosition(double position) {
    sim.setPositionSi(position / unitsPerSi);
    setpoint = position;
    controller.reset();
  }

  @Override
  public void setCurrentLimit(double amps) {
    currentLimitAmps = amps;
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    brakeMode = enabled;
  }

  public boolean isBrakeMode() {
    return brakeMode;
  }
}
