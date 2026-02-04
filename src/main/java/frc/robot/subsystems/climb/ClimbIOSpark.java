package frc.robot.subsystems.climb;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ClimbIOSpark implements ClimbIO{
 
    // Hardware objects
    private final SparkMax climb;

    private final RelativeEncoder climbEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController climbController;

    public ClimbIOSpark(){

    climb = new SparkMax(0, MotorType.kBrushless);

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
        .maxAcceleration(0)
        .cruiseVelocity(0)
        .allowedProfileError(0);
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
    //     // Update drive inputs
    // sparkStickyFault = false;
    // ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    // ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    // ifOk(
    //     driveSpark,
    //     new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
    //     (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    // ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    // inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // if ((Math.abs((canTurnEncoder.getAbsolutePosition().getValue().in(Degree) - (relTurnEncoder.getPosition() - zeroRotation.getDegrees())))) > 5) {
    //     // tryUntilOk(turnSpark, 1, () -> relTurnEncoder.setPosition(canTurnEncoder.getAbsolutePosition().getValue().in(Radians)));
    //     // System.out.println("ROTATE!");
    // }

    // // Update turn inputs
    // sparkStickyFault = false;
    // ifOk(
    //     turnSpark,
    //     relTurnEncoder::getPosition,
    //     (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));

    // ifOk(turnSpark, relTurnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    // ifOk(
    //     turnSpark,
    //     new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
    //     (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    // ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    // inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // inputs.canPosition = new Rotation2d(canTurnEncoder.getAbsolutePosition().getValue().in(Radians));
    // // Update odometry inputs
    // inputs.odometryTimestamps =
    //     timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryDrivePositionsRad =
    //     drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryTurnPositions =
    //     turnPositionQueue.stream()
    //         .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
    //         .toArray(Rotation2d[]::new);
    // timestampQueue.clear();
    // drivePositionQueue.clear();
    // turnPositionQueue.clear();
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

    SparkUtil.tunePID(
        "Climb PID " + module, 
        climb, 
        climbConfig, 
        new double [] {climbKp, climbKi, climbKd, 0, climbKf, 0, 0},
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters,
        true,
        false
        );
  }



