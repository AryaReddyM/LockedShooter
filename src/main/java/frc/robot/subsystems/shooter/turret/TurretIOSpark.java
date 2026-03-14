package frc.robot.subsystems.shooter.turret;

import static frc.robot.util.SparkUtil.ifOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretIOSpark implements TurretIO {

    // Hardware objects
    private final SparkMax turret;

    private final RelativeEncoder turretAbsoluteEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController turretController;

    private double latencyCompensatedMS = TurretConstants.latencyComepnsationMS;
    private double desiredPos = 0.0;
    private boolean turretDisabled = false;

    public TurretIOSpark() {
        turret = new SparkMax(TurretConstants.kTurretCanId, MotorType.kBrushless);

        turretAbsoluteEncoder = turret.getEncoder();

        turretController = turret.getClosedLoopController();

        // Configure turret motor
        // NEED TO CONFIGURE
        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig
                .inverted(TurretConstants.kTurretinverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(TurretConstants.kTurretCurrentLimit)
                .voltageCompensation(12.0);
        turretConfig.encoder
                .positionConversionFactor(TurretConstants.kTurretPositionConversionFactor)
                .velocityConversionFactor(TurretConstants.kTurretVelocityConversionFactor)
                .quadratureAverageDepth(10)
                .quadratureMeasurementPeriod(2);
        turretConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingInputRange(TurretConstants.kBackwardSoftLimit, TurretConstants.kForwardSoftLimit)
                .positionWrappingEnabled(false)
                .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD).maxMotion
                .maxAcceleration(TurretConstants.kTurretMaxAccel)
                .cruiseVelocity(TurretConstants.kTurretCruiseVel)
                .allowedProfileError(TurretConstants.kTurretDeviationErr);
        turretConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(10)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        turretConfig
            .softLimit
            .forwardSoftLimit(TurretConstants.kForwardSoftLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.kBackwardSoftLimit)
            .reverseSoftLimitEnabled(true);

        turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turret.clearFaults();

        SparkUtil.tunePID(
                "Turret",
                turret,
                turretConfig,
                new double[] { TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD,
                        TurretConstants.kTurretS, TurretConstants.kTurretV, TurretConstants.kTurretA,
                        TurretConstants.kTurretG, TurretConstants.kTurretMaxAccel, TurretConstants.kTurretCruiseVel,
                        TurretConstants.kTurretDeviationErr },
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
                true,
                true);

        DogLog.tunable("Turret/Latency", TurretConstants.latencyComepnsationMS, (newVal) -> {
            latencyCompensatedMS = newVal;
        });

        SmartDashboard.putData("Turret/Zero", new InstantCommand(() -> {
            turretAbsoluteEncoder.setPosition(0);
        }));

        SmartDashboard.putData("Turret/Disable", new InstantCommand(() -> {
            turretDisabled = true;
        }));
        SmartDashboard.putData("Turret/Enable", new InstantCommand(() -> {
            turretDisabled = false;
            
        }));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretVelRadPerSec = turretAbsoluteEncoder.getVelocity();

        ifOk(turret, turretAbsoluteEncoder::getVelocity, (value) -> inputs.turretVelRadPerSec = value);

        ifOk(
                turret,
                turretAbsoluteEncoder::getPosition,
                (value) -> inputs.turretRotation2d = new Rotation2d(
                        value + inputs.turretVelRadPerSec * latencyCompensatedMS / 1000).minus(TurretConstants.kTurretAbsEncoderOffset));

        ifOk(
                turret,
                new DoubleSupplier[] { turret::getAppliedOutput, turret::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(turret, turret::getOutputCurrent, (value) -> inputs.currentAmps = value);

        inputs.desiredPos = this.desiredPos;
    }

    @Override
    public void setTurretVoltage(double volts) {
        turret.setVoltage(volts);
    }

    @Override
    public void setTurretPosition(double position, double ff) {
        if (turretDisabled) {
            return;
        }

        position = MathUtil.inputModulus(position, TurretConstants.kBackwardSoftLimit, TurretConstants.kForwardSoftLimit);
        this.desiredPos = position;
        turretController.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0 ,ff, ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurretPosition(double position) {
        if (turretDisabled) {
            return;
        }
        position = MathUtil.inputModulus(position, TurretConstants.kBackwardSoftLimit, TurretConstants.kForwardSoftLimit);
        this.desiredPos = position;
        turretController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public double getTurretPosition() {
        return turretController.getMAXMotionSetpointPosition();
    }

    @Override
    public double getTurretVelocity() {
        return turretController.getMAXMotionSetpointVelocity();
    }

    @Override
    public Rotation2d getRobotToTurretRotation() {
        double turretPos = getTurretPosition();
        // conversion Factor
        return new Rotation2d(turretPos);
    }

    @Override
    public void stopTurret() {
        turret.stopMotor();
    }
}
