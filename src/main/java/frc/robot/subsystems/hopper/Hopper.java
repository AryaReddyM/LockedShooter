package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.GetTuned;
import frc.robot.util.state.StateMachine;

public class Hopper extends StateMachine<Hopper.State> implements HopperIO {

    private final RobotState state;
    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    private double spinRadians = 0.0;

    private final Timer jammedTimer = new Timer();
    private final Timer unJamTimer = new Timer();
    private boolean jammed = false;

    private Consumer<Object> override;

    public Hopper(HopperIO hopperIO, RobotState state) {
        super("Hopper", State.UNDETERMINED, State.class);
        this.hopperIO = hopperIO;
        this.state = state;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        hopperIO.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);

        spinRadians = spinRadians + (0.02 * HopperConstants.kHopperShootSpeed / HopperConstants.kRollerRadiusMeters);

        Logger.recordOutput("Hopper/Pose",
                new Pose3d()
                        .plus(HopperConstants.hopperOrigin)
                        .plus(new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0, 0, spinRadians))));
        if (jammed) {
              outake();
        } else if (override != null) {
            override.accept(null);
        } else if (getState() == State.SHOOT) {
            if (state.getShooter().getFlywheel().isReady()) { // && state.getShooter().getTurret().isReady()) {
                shoot();
            } else {
                stop();
            }
        } else if (getState() == State.OUTAKE) {
            outake();
        } else {
            stop();
        }
        Logger.recordOutput("Hopper/Overriden", override != null);
        updateJamDetection();
    }

    public void shoot() {
        hopperIO.setHopperSpeed(GetTuned.getNumber("Hopper/Shoot Speed", HopperConstants.kHopperShootSpeed));
    }

    public void outake() {
        hopperIO.setHopperSpeed(GetTuned.getNumber("Hopper/Outtake Speed", HopperConstants.kHopperOuttakeSpeed));
    }

    public void stop() {
        hopperIO.setHopperSpeed(0);
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.UNDETERMINED, State.IDLE, State.OUTAKE, State.SHOOT);
    }

    private void registerStateCommands() {
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
    }

    private void updateJamDetection() {
        if (jammedTimer.isRunning() && jammedTimer.hasElapsed(Seconds.of(0.2))) {
            jammed = true;
            unJamTimer.start();
            jammedTimer.stop();
            jammedTimer.reset();
        }
        if (jammed && unJamTimer.isRunning() && unJamTimer.hasElapsed(0.1)) {
            jammed = false;
            unJamTimer.stop();
            unJamTimer.reset();
        }
        if (Math.abs(hopperIO.getHopperSpeed()) < (GetTuned.getNumber("Hopper/Jam Speed", HopperConstants.kHopperJamTol))
                && getState() == State.SHOOT
                && !jammed) {
            jammedTimer.start();
        } else if (jammedTimer.isRunning()) {
            jammedTimer.stop();
            jammedTimer.reset();
        }
    }

    public enum State {
        UNDETERMINED,

        IDLE,
        OUTAKE,
        SHOOT

        // flags

    }

}
