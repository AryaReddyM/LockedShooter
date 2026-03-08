package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.GetTuned;
import frc.robot.util.RobotTime;
import frc.robot.util.ShooterSetpoint;
import frc.robot.util.TurretVisualizer;
import frc.robot.util.state.StateMachine;

public class Turret extends StateMachine<Turret.State> implements TurretIO {
    private final RobotState state;
    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretVisualizer turretVisualizer;
    private final Timer timer;

    private double tunedSetpoint = 0.0;
    private Consumer<Object> override;

    public Turret(TurretIO turretIO, RobotState state) {
        super("Turret", State.UNDETERMINED, State.class);
        this.turretIO = turretIO;
        this.state = state;

        this.timer = new Timer();
        this.turretVisualizer = new TurretVisualizer(() -> {
            return inputs.robotTurretPos;
        }, () -> {
            return state.getLatestMeasuredFieldRelativeChassisSpeeds();
        });

        timer.start();

        DogLog.tunable("Turret/Custom Setpoint", tunedSetpoint, newVal -> tunedSetpoint = newVal);

        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        turretIO.updateInputs(inputs);
        {
            if (state.robotState == 2) {

                if (state.getShooter().getState() == Shooter.State.PASSING
                        || state.getShooter().getState() == Shooter.State.PASS_TRACKING) {
                    turretVisualizer.updateFuel(state,
                            MetersPerSecond.of(state.getCurrentPassSetpoint().getShooterRPS()),
                            Degrees.of(90).minus(Radians.of(state.getCurrentPassSetpoint().getHoodRadians())));
                } else {
                    turretVisualizer.updateFuel(state,
                            MetersPerSecond.of(state.getCurrentHubSetpoint().getShooterRPS()),
                            Degrees.of(90).minus(Radians.of(state.getCurrentHubSetpoint().getHoodRadians())));
                }

                if ((state.getShooter().getState() == Shooter.State.SHOOTING || state.getShooter().getState() == Shooter.State.PASSING) && state.getSimFuelCount() > 0
                        && (timer.hasElapsed(0.08))) {
                    state.setSimFuelCount(state.getSimFuelCount() - 1);
                    timer.reset();

                    ShooterSetpoint currentSetpoint = (state.getShooter().getState() == Shooter.State.SHOOTING) ? state.getCurrentHubSetpoint() : state.getCurrentPassSetpoint();
                    state.getFuelSim().launchFuel(
                            MetersPerSecond.of(currentSetpoint.getShooterRPS()),
                            Degrees.of(90).minus(Radians.of(currentSetpoint.getHoodRadians())),
                            Radians.of(currentSetpoint.getTurretRadiansFromCenter()),
                        Meters.of(VisionConstants.kTurretToRobotCenter.getTranslation().getZ()));
                }
            }
            inputs.robotTurretPos = new Pose3d(
                    state.getLatestFieldToRobot().getValue().getX(),
                    state.getLatestFieldToRobot().getValue().getY(),
                    0.0,
                    new Rotation3d(0.0, 0.0,
                            state.getLatestFieldToRobot().getValue().getRotation().plus(Rotation2d.fromRadians(inputs.desiredPos)).getRadians()));
        }
        Logger.processInputs("Turret", inputs);

        { // TURRET POS SETTER
            if (override != null) {
                override.accept(null);
            } else if (getState() == State.HUB_TRACKING) {
                setPos(state.getCurrentHubSetpoint().getTurretRadiansFromCenter(),
                        state.getCurrentHubSetpoint().getTurretFF());
            } else if (getState() == State.PASS_TRACKING) {
                setPos(state.getCurrentPassSetpoint().getTurretRadiansFromCenter(),
                        state.getCurrentPassSetpoint().getTurretFF());
            } else if (getState() == State.TUNING) {
                setPos(tunedSetpoint);
            } else {
                stop();
            }
        }

        state.addTurretUpdates(RobotTime.getTimestampSeconds(), inputs.turretRotation2d, //inputs.turretRotation2d
                inputs.turretPos,
                inputs.turretVelRadPerSec);

        Logger.recordOutput("Turret/Pose",
                new Pose3d()
                        .plus(VisionConstants.kTurretToRobotCenter)
                        .plus(new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0, 0, inputs.desiredPos)
                        )));
        Logger.recordOutput("Turret/Overriden", override!=null);
    }

    public void setPos(double position, double ff) {
        turretIO.setTurretPosition(position, ff);
    }

    public void setPos(double position) {
        turretIO.setTurretPosition(position);
    }

    public void stop() {
        turretIO.stopTurret();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.UNDETERMINED, State.TUNING);
    }

    private void registerStateCommands() {
    }

    public Command waitForShootReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentHubSetpoint().getTurretRadiansFromCenter()
                    - turretIO.getTurretPosition()) < tolerance;
        });
    }

    public Command waitForPassReady(double tolerance) {
        return new WaitUntilCommand(() -> {
            return Math.abs(state.getCurrentPassSetpoint().getTurretRadiansFromCenter()
                    - turretIO.getTurretPosition()) < tolerance;
        });
    }

    public boolean isReady() {
        return true;
        // boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        //     Pose2d currentPose = state.getLatestFieldToRobot().getValue();

        //     double blueHubX = VisionConstants.FieldConstants.HUB_BLUE.getX();
        //     double redHubX = VisionConstants.FieldConstants.HUB_RED.getX();

        //     boolean shouldShoot = isBlue
        //             ? currentPose.getX() <= blueHubX
        //             : currentPose.getX() >= redHubX;

        //     if (shouldShoot && RobotState.hubActivated.get()) {
        //         return Math.abs(state.getCurrentHubSetpoint().getTurretRadiansFromCenter()
        //             - turretIO.getTurretPosition()) < Units.degreesToRadians(GetTuned.getNumber("Turret/Tolerance", TurretConstants.kReadyToleranceDegrees));
        //     } else {
        //         return Math.abs(state.getCurrentPassSetpoint().getTurretRadiansFromCenter()
        //             - turretIO.getTurretPosition()) < Units.degreesToRadians(GetTuned.getNumber("Turret/Tolerance", TurretConstants.kReadyToleranceDegrees));
        //     }
    }

    public Rotation2d getRotation() {
        return inputs.turretRotation2d;
    }

    public double getDesiredPos() {
        return inputs.desiredPos;
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
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