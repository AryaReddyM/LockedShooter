package frc.robot.subsystems.shooter.flywheel;


import static edu.wpi.first.units.Units.Meters;
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

import dev.doglog.DogLog;
import dev.doglog.internal.tunable.Tunable;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.BangBangController;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class FlywheelIOSpark implements FlywheelIO{
 
    // Hardware objects
    private final SparkFlex flywheel;
    private final SparkFlex flywheelFollower;
    private final BangBangController flywheelBangBangController;


    @SuppressWarnings("unused")
    private final RelativeEncoder flywheelEncoder;


    // Closed loop controllers
    private final SparkClosedLoopController flywheelController;

    public FlywheelIOSpark(){

    flywheel = new SparkFlex(FlywheelConstants.kFlywheelCanID, MotorType.kBrushless);
    flywheelFollower = new SparkFlex(FlywheelConstants.kFlywheelFollowerCanID, MotorType.kBrushless);

    flywheelEncoder = flywheel.getEncoder();

    flywheelController = flywheel.getClosedLoopController();
    flywheelBangBangController = new BangBangController(); // need to add a tolerance
;

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
        .outputRange(0,1)
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

     flywheelConfig
        .closedLoop
        .feedForward
        .kS(FlywheelConstants.kFlywheelS)
        .kV(FlywheelConstants.kFlywheelV)
        .kA(FlywheelConstants.kFlywheelA)
        .kG(FlywheelConstants.kFlywheelG);

    flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheel.clearFaults();

    SparkFlexConfig flywheelFollowerConfig = new SparkFlexConfig();
    flywheelFollowerConfig
        .follow(FlywheelConstants.kFlywheelCanID);

    flywheelFollower.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheel.clearFaults();

    DogLog.tunable("Flywheel/Tolerance", FlywheelConstants.kFlywheelSpeedTolerance, (a) -> {
        flywheelBangBangController.setTolerance(a);
    });


    SparkUtil.tunePID(
        "Flywheel",
        flywheel,
        flywheelConfig,
        new double[] {FlywheelConstants.kFlywheelP, FlywheelConstants.kFlywheelI, FlywheelConstants.kFlywheelD, FlywheelConstants.kFlywheelS, FlywheelConstants.kFlywheelV, FlywheelConstants.kFlywheelA, FlywheelConstants.kFlywheelG, FlywheelConstants.kFlywheelMaxAccel, FlywheelConstants.kFlywheelCruiseVel, FlywheelConstants.kFlywheelDeviationErr},
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters,
        true,
        true);

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        ifOk(flywheel, flywheelEncoder::getPosition, (value) -> inputs.posRad = value);
        ifOk(flywheel, flywheelEncoder::getVelocity, (value) -> inputs.velPerSec = value);
        ifOk(flywheel, flywheelEncoder::getVelocity, (value) -> inputs.omegaRadPerSec = value / FlywheelConstants.kFlywheelRadius.in(Meters));
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
        // setFlywheelSpeed(speed);
        flywheelController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    }

    @Override
    public void setFlywheelSpeed(double speed) {
        flywheelController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
        // double currentVelocity = flywheelEncoder.getVelocity();
        // double bangBangOutput = flywheelBangBangController.calculate(currentVelocity, speed);
        // double ff = FlywheelConstants.kFlywheelS + FlywheelConstants.kFlywheelV * speed;
        // flywheel.setVoltage(bangBangOutput * 12.0 + ff);
    }

    @Override
    public void stopFlywheel() {
        flywheel.stopMotor();
    }

    @Override
    public boolean isAtSpeed(double speed, double tolerance) {
        // if (speed < 1) {
        //     return false;
        // }
        // double currentVelocity = flywheelEncoder.getVelocity();
        return true;
        // return Math.abs(currentVelocity - speed) < tolerance;
        // return flywheelBangBangController.atSetpoint();
    }
  }



