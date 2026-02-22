package frc.robot.subsystems.climb;

import static frc.robot.util.SparkUtil.ifOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimbIOSpark implements ClimbIO {

    private final SparkFlex climb;
    private final RelativeEncoder climbEncoder;
    private double desiredPos = 0.0;
    private final SparkClosedLoopController climbController;
    private final SparkMaxConfig climbConfig;

    public ClimbIOSpark() {

        climb = new SparkFlex(ClimbConstants.kClimbCanID, MotorType.kBrushless);
        climbEncoder = climb.getEncoder();
        climbController = climb.getClosedLoopController();

        climbConfig = new SparkMaxConfig();
        
        climbConfig
                .inverted(ClimbConstants.kClimbinverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ClimbConstants.kClimbCurrentLimit)
                .voltageCompensation(12.0);

        climbConfig.encoder
                .positionConversionFactor(ClimbConstants.kClimbPositionConversionFactor)
                .velocityConversionFactor(ClimbConstants.kClimbVelocityConversionFactor);

        climbConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ClimbConstants.kClimbP, ClimbConstants.kClimbI, ClimbConstants.kClimbD, ClosedLoopSlot.kSlot0)
                .pid(ClimbConstants.kClimbActionP, ClimbConstants.kClimbI, ClimbConstants.kClimbD, ClosedLoopSlot.kSlot1);

        climbConfig.closedLoop.maxMotion
                .maxAcceleration(ClimbConstants.kClimbMaxAccel, ClosedLoopSlot.kSlot0)
                .cruiseVelocity(ClimbConstants.kClimbCruiseVel, ClosedLoopSlot.kSlot0)
                .maxAcceleration(ClimbConstants.kClimbActionMaxAccel, ClosedLoopSlot.kSlot1)
                .cruiseVelocity(ClimbConstants.kClimbActionCruiseVel, ClosedLoopSlot.kSlot1)
                .allowedProfileError(ClimbConstants.kClimbDeviationErr, ClosedLoopSlot.kSlot0)
                .allowedProfileError(ClimbConstants.kClimbDeviationErr, ClosedLoopSlot.kSlot1);

        climbConfig.signals
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
        ifOk(climb, climbEncoder::getPosition, (value) -> inputs.posRad = value);
        ifOk(climb, climbEncoder::getVelocity, (value) -> inputs.velPerSec = value);
        ifOk(
                climb,
                new DoubleSupplier[] { climb::getAppliedOutput, climb::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(climb, climb::getOutputCurrent, (value) -> inputs.currentAmps = value);
        inputs.desiredPos = desiredPos;
    }

    @Override
    public void setClimbVoltage(double volts) {
        climb.setVoltage(volts);
    }

    @Override
    public void setClimbPosition(double position) {
        setClimbPosition(position, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setClimbPosition(double position, ClosedLoopSlot slot) {
        this.desiredPos = position;
        climbController.setSetpoint(position, ControlType.kMAXMotionPositionControl, slot);
    }

    @Override
    public void stopClimb() {
        climb.stopMotor();
    }

    @Override
    public void setCurrentLimit(double limit) {
        climbConfig.smartCurrentLimit((int) limit);
        climb.configure(climbConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setMotorOutput(double output) {
        climb.set(output);
    }

    @Override
    public double getMotorCurrent() {
        return climb.getOutputCurrent();
    }

    @Override
    public void zeroEncoder() {
        climbEncoder.setPosition(0);
    }
}