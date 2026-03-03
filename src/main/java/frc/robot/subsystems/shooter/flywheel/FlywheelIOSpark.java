package frc.robot.subsystems.shooter.flywheel;


import static frc.robot.util.SparkUtil.ifOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class FlywheelIOSpark implements FlywheelIO{
 
    // Hardware objects
    private final SparkFlex flywheel;
    private final SparkFlex flywheelFollower;

    @SuppressWarnings("unused")
    private final RelativeEncoder flywheelEncoder;


    // Closed loop controllers
    private final SparkClosedLoopController flywheelController;

    public FlywheelIOSpark(){

    flywheel = new SparkFlex(FlywheelConstants.kFlywheelCanID, MotorType.kBrushless);
    flywheelFollower = new SparkFlex(FlywheelConstants.kFlywheelFollowerCanID, MotorType.kBrushless);

    flywheelEncoder = flywheel.getEncoder();

    flywheelController = flywheel.getClosedLoopController();

    // Configure extention motor
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    flywheelConfig
        .inverted(FlywheelConstants.kFlywheelinverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(FlywheelConstants.kFlywheelCurrentLimit)
        .voltageCompensation(12.0);
    flywheelConfig
        .encoder
        .positionConversionFactor(FlywheelConstants.kFlywheelPositionConversionFactor)
        .velocityConversionFactor(FlywheelConstants.kFlywheelVelocityConversionFactor);
    flywheelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(FlywheelConstants.kFlywheelP, FlywheelConstants.kFlywheelI, FlywheelConstants.kFlywheelD)
        .maxMotion
        .maxAcceleration(FlywheelConstants.kFlywheelMaxAccel)
        .cruiseVelocity(FlywheelConstants.kFlywheelCruiseVel)
        .allowedProfileError(FlywheelConstants.kFlywheelDeviationErr);
    flywheelConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheel.clearFaults();

    SparkFlexConfig flywheelFollowerConfig = new SparkFlexConfig();
    flywheelFollowerConfig
        .follow(FlywheelConstants.kFlywheelCanID);

    // flywheelFollower.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheel.clearFaults();




    SparkUtil.tunePID(
        "Flywheel",
        flywheel,
        flywheelConfig,
        new double[] {FlywheelConstants.kFlywheelP, FlywheelConstants.kFlywheelI, FlywheelConstants.kFlywheelD, 0,0,0,0, FlywheelConstants.kFlywheelMaxAccel, FlywheelConstants.kFlywheelCruiseVel, FlywheelConstants.kFlywheelDeviationErr},
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters,
        false,
        true);

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        ifOk(flywheel, flywheelEncoder::getPosition, (value) -> inputs.posRad = value);
        ifOk(flywheel, flywheelEncoder::getVelocity, (value) -> inputs.velPerSec = value);
        ifOk(
                flywheel,
                new DoubleSupplier[] { flywheel::getAppliedOutput, flywheel::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(flywheel, flywheel::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        flywheel.setVoltage(volts);
    }

    @Override
    public void setFlywheelSpeed(double speed, double ff) {
        flywheelController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    }

    @Override
    public void setFlywheelSpeed(double speed) {
        flywheelController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void stopFlywheel() {
        flywheel.stopMotor();
    }

    @Override
    public boolean isAtSpeed(double speed, double tolerance) {
        if (Math.abs(flywheelController.getSetpoint() - speed) < tolerance) {
            return true;
        }
        return false;
    }
  }



