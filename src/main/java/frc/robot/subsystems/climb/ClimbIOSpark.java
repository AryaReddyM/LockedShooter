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
        .inverted(climbInverted) 
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(climbMotorCurrentLimit)
        .voltageCompensation(12.0);
    climbConfig
        .encoder
        .positionConversionFactor(climbEncoderPositionFactor)
        .velocityConversionFactor(climbEncoderVelocityFactor);
    climbConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(climbKp, climbKi, climbKd)
        .maxMotion
        .maxAcceleration(climbMaxAcceleration)
        .cruiseVelocity(climbCruiseVelocity)
        .allowedProfileError(climbAllowedProfileError);
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

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        
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
