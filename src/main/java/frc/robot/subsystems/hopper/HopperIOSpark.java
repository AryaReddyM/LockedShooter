package frc.robot.subsystems.hopper;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class HopperIOSpark implements HopperIO{
 
    // Hardware objects
    private final SparkFlex hopper;

    private final RelativeEncoder hopperEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController hopperController;

    public HopperIOSpark(){

    hopper = new SparkFlex(HopperConstants.kHopperCanID, MotorType.kBrushless);

    hopperEncoder = hopper.getEncoder();

    hopperController = hopper.getClosedLoopController();

    // Configure extention motor
    SparkFlexConfig hopperConfig = new SparkFlexConfig();
    hopperConfig
        .inverted(HopperConstants.kHopperinverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HopperConstants.kHopperCurrentLimit)
        .voltageCompensation(12.0);
    hopperConfig
        .encoder
        .positionConversionFactor(HopperConstants.kHopperPositionConversionFactor)
        .velocityConversionFactor(HopperConstants.kHopperVelocityConversionFactor)
        .quadratureAverageDepth(10)
        .quadratureMeasurementPeriod(2);
    hopperConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(HopperConstants.kHopperP, HopperConstants.kHopperI, HopperConstants.kHopperD)
        .maxMotion
        .maxAcceleration(HopperConstants.kHopperMaxAccel)
        .cruiseVelocity(HopperConstants.kHopperCruiseVel)
        .allowedProfileError(HopperConstants.kHopperDeviationErr);
    // hopperConfig
    //     .signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);

    hopper.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hopper.clearFaults();

    // SparkUtil.tunePID(
    //     "Hopper",
    //     hopper,
    //     hopperConfig,
    //     new double[] {HopperConstants.kHopperP, HopperConstants.kHopperI, HopperConstants.kHopperD, 0,0,0,0, HopperConstants.kHopperMaxAccel, HopperConstants.kHopperCruiseVel, HopperConstants.kHopperDeviationErr},
    //     ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters,
    //     false,
    //     true);

    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        ifOk(hopper, hopperEncoder::getPosition, (value) -> inputs.posRad = value);
        ifOk(hopper, hopperEncoder::getVelocity, (value) -> inputs.velPerSec = value);
        ifOk(
                hopper,
                new DoubleSupplier[] { hopper::getAppliedOutput, hopper::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(hopper, hopper::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    @Override
    public void setHopperVoltage(double volts) {
        hopper.setVoltage(volts);
    }

    @Override
    public void setHopperSpeed(double speed) {
        hopperController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void stopHopper() {
        hopper.stopMotor();
    }
  }



