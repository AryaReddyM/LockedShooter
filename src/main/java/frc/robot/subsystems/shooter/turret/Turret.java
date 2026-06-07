package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.ServoMotorSubsystem;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.logging.GetTuned;
import frc.robot.util.field.RobotTime;
import frc.robot.util.shooting.ShooterSetpoint;
import frc.robot.util.shooting.TurretVisualizer;

public class Turret extends ServoMotorSubsystem {
    private final RobotState state;
    private final TurretVisualizer turretVisualizer;
    private final Timer timer;

    private double tunedSetpoint = 0.0;
    private double latencyMs = TurretConstants.latencyComepnsationMS;
    private double desiredPos = 0.0;
    private Rotation2d turretRotation2d = new Rotation2d();
    private Pose3d robotTurretPos = new Pose3d();
    private Consumer<Object> override;
    private State stateValue = State.IDLE;

    public Turret(MotorIO io, RobotState state) {
        super(io, "Turret", TurretConstants.kTurretDeviationErr);
        this.state = state;

        this.timer = new Timer();
        this.turretVisualizer = new TurretVisualizer(() -> robotTurretPos,
                () -> state.getLatestMeasuredFieldRelativeChassisSpeeds());

        timer.start();

        DogLog.tunable("Turret/Custom Setpoint", tunedSetpoint, newVal -> tunedSetpoint = newVal);
        DogLog.tunable("Turret/Latency", latencyMs, newVal -> latencyMs = newVal);

        { // tof tuner
            for (double distance : TurretConstants.LOGGED_DISTANCES) {
                double tof = TurretConstants.TOF_MAP.get(distance);
                DogLog.tunable("TOF Tuning/" + distance, tof, newtof -> {
                    TurretConstants.TOF_MAP.put(distance, newtof);
                });
            }
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        double turretPosRad = getPositionRad();
        double turretVel = getVelocityRadPerSec();
        turretRotation2d = new Rotation2d(turretPosRad + turretVel * latencyMs / 1000.0)
                .minus(TurretConstants.kTurretAbsEncoderOffset);

        {
            if (Constants.currentMode == Constants.Mode.SIM) {
                Superstructure.State superstructureState = state.getSuperstructure().getState();

                if (superstructureState == Superstructure.State.PASSING
                        || superstructureState == Superstructure.State.PASSING_INTAKING
                        || superstructureState == Superstructure.State.PASS_TRACKING) {
                    turretVisualizer.updateFuel(state,
                            MetersPerSecond.of(state.getCurrentPassSetpoint().getShooterRPS()),
                            Degrees.of(90).minus(Radians.of(state.getCurrentPassSetpoint().getHoodRadians())));
                } else {
                    turretVisualizer.updateFuel(state,
                            MetersPerSecond.of(state.getCurrentHubSetpoint().getShooterRPS()),
                            Degrees.of(90).minus(Radians.of(state.getCurrentHubSetpoint().getHoodRadians())));
                }

                if ((superstructureState == Superstructure.State.SHOOTING
                        || superstructureState == Superstructure.State.SHOOTING_INTAKING
                        || superstructureState == Superstructure.State.PASSING
                        || superstructureState == Superstructure.State.PASSING_INTAKING) && state.getSimFuelCount() > 0
                        && (timer.hasElapsed(0.08))) {
                    state.setSimFuelCount(state.getSimFuelCount() - 1);
                    timer.reset();

                    ShooterSetpoint currentSetpoint = (superstructureState == Superstructure.State.SHOOTING
                            || superstructureState == Superstructure.State.SHOOTING_INTAKING)
                            ? state.getCurrentHubSetpoint()
                            : state.getCurrentPassSetpoint();
                    state.getFuelSim().launchFuel(
                            MetersPerSecond.of(currentSetpoint.getShooterRPS()),
                            Degrees.of(90).minus(Radians.of(currentSetpoint.getHoodRadians())),
                            Radians.of(currentSetpoint.getTurretRadiansFromCenter()),
                            Meters.of(VisionConstants.kTurretToRobotCenter.getTranslation().getZ()));
                }
            }
            robotTurretPos = new Pose3d(
                    state.getLatestFieldToRobot().getValue().getX(),
                    state.getLatestFieldToRobot().getValue().getY(),
                    0.0,
                    new Rotation3d(0.0, 0.0,
                            state.getLatestFieldToRobot().getValue().getRotation()
                                    .plus(Rotation2d.fromRadians(desiredPos)).getRadians()));
        }

        { // TURRET POS SETTER
            if (override != null) {
                override.accept(null);
            } else if (stateValue == State.HUB_TRACKING) {
                setPos(state.getCurrentHubSetpoint().getTurretRadiansFromCenter(),
                        state.getCurrentHubSetpoint().getTurretFF());
            } else if (stateValue == State.PASS_TRACKING) {
                setPos(state.getCurrentPassSetpoint().getTurretRadiansFromCenter(),
                        state.getCurrentPassSetpoint().getTurretFF());
            } else if (stateValue == State.TUNING) {
                setPos(tunedSetpoint);
            } else {
                stop();
            }
        }

        state.addTurretUpdates(RobotTime.getTimestampSeconds(), turretRotation2d, turretPosRad, turretVel);

        Logger.recordOutput("Turret/Desired", desiredPos);
        Logger.recordOutput("Turret/Pose",
                new Pose3d()
                        .plus(VisionConstants.kTurretToRobotCenter)
                        .plus(new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0, 0, desiredPos))));
        Logger.recordOutput("Turret/Overriden", override != null);
        Logger.recordOutput("Turret/State", stateValue.toString());
    }

    public void setPos(double position, double ff) {
        position = MathUtil.inputModulus(position, TurretConstants.kBackwardSoftLimit, TurretConstants.kForwardSoftLimit);
        desiredPos = position;
        applySetpoint(Setpoint.motionMagicPosition(position), ff);
    }

    public void setPos(double position) {
        position = MathUtil.inputModulus(position, TurretConstants.kBackwardSoftLimit, TurretConstants.kForwardSoftLimit);
        desiredPos = position;
        applySetpoint(Setpoint.motionMagicPosition(position));
    }

    @Override
    public void stop() {
        super.stop();
    }

    public Command waitForShootReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentHubSetpoint().getTurretRadiansFromCenter()
                    - getPositionRad()) < tolerance;
        });
    }

    public Command waitForPassReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentPassSetpoint().getTurretRadiansFromCenter()
                    - getPositionRad()) < tolerance;
        });
    }

    public boolean isReady() {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        Pose2d currentPose = state.getLatestFieldToRobot().getValue();

        double blueHubX = VisionConstants.FieldConstants.HUB_BLUE.getX();
        double redHubX = VisionConstants.FieldConstants.HUB_RED.getX();

        boolean shouldShoot = isBlue
                ? currentPose.getX() <= blueHubX
                : currentPose.getX() >= redHubX;

        if (shouldShoot) {
            Logger.recordOutput("Turret/Ready", Math.abs(state.getCurrentHubSetpoint().getTurretRadiansFromCenter()
                    - getPositionRad()) < Units.degreesToRadians(
                            GetTuned.getNumber("Turret/Tolerance", TurretConstants.kReadyToleranceDegrees)));
        } else {
            Logger.recordOutput("Turret/Ready", Math.abs(state.getCurrentPassSetpoint().getTurretRadiansFromCenter()
                    - getPositionRad()) < Units.degreesToRadians(
                            GetTuned.getNumber("Turret/Tolerance", TurretConstants.kReadyToleranceDegrees)));
        }

        return true;
    }

    public Rotation2d getRotation() {
        return turretRotation2d;
    }

    public double getDesiredPos() {
        return desiredPos;
    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
    }

    public State getState() {
        return stateValue;
    }

    public void requestTransition(State state) {
        stateValue = state == State.UNDETERMINED ? State.IDLE : state;
    }

    public Command transitionCommand(State state) {
        return runOnce(() -> requestTransition(state));
    }

    public enum State {
        UNDETERMINED,

        IDLE,
        HUB_TRACKING,
        PASS_TRACKING,
        TUNING

        // flags

    }

}
