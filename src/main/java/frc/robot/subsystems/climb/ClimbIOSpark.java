package frc.robot.subsystems.climb;

import static frc.robot.util.SparkUtil.ifOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimbIOSpark implements ClimbIO {

    // Hardware objects
    private final SparkMax climb;

    private final RelativeEncoder climbEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController climbController;

    public ClimbIOSpark() {

        climb = new SparkMax(ClimbConstants.kClimbCanID, MotorType.kBrushless);

        climbEncoder = climb.getEncoder();

        climbController = climb.getClosedLoopController();

    // Configure extention motor
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    
    climbConfig 
        .inverted(ClimbConstants.climbInverted) 
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimbConstants.climbMotorCurrentLimit)
        .voltageCompensation(12.0);
    climbConfig
        .encoder
        .positionConversionFactor(ClimbConstants.climbEncoderPositionFactor)
        .velocityConversionFactor(ClimbConstants.climbEncoderVelocityFactor);
    climbConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(ClimbConstants.climbKp, ClimbConstants.climbKi, ClimbConstants.climbKd)
        .maxMotion
        .maxAcceleration(ClimbConstants.climbMaxAcceleration)
        .cruiseVelocity(ClimbConstants.climbCruiseVelocity)
        .allowedProfileError(ClimbConstants.climbAllowedProfileError);
    climbConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

        climb.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climb.clearFaults();

    SparkUtil.tunePID(
        "Climb PID ", 
        climb, 
        climbConfig, 
        new double [] {ClimbConstants.climbKp, ClimbConstants.climbKi, ClimbConstants.climbKd, 0, ClimbConstants.climbKf, 0, 0},
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters,
        false,
        true
        );
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        ifOk(climb, climbEncoder::getPosition, (value) -> inputs.climbPositionRad = value);
        ifOk(climb, climbEncoder::getVelocity, (value) -> inputs.climbVelocityRadPerSec = value);
        ifOk(
                climb,
                new DoubleSupplier[] { climb::getAppliedOutput, climb::getBusVoltage },
                (values) -> inputs.climbAppliedVolts = values[0] * values[1]);
        ifOk(climb, climb::getOutputCurrent, (value) -> inputs.climbCurrentAmps = value);
    }

    @Override
    public void setClimbVoltage(double volts) {
        climb.setVoltage(volts);
    }

    @Override
    public void setClimbPosition(double position) {
        climbController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stopClimb() {
        climb.stopMotor();
    }
}
