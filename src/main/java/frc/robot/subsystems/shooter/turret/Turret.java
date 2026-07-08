package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Turret extends StateMachine<Turret.State> {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private final RobotState state;

  private double tunedSetpoint = 0.0;
  private double desiredPos = 0.0;

  public Turret(MotorIO io, RobotState state) {
    super("Turret", State.UNDETERMINED, State.class);
    this.io = io;
    this.state = state;

    addOmniTransitions(State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.TUNING);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    switch (getState()) {
      case HUB_TRACKING ->
          setPos(
              state.getCurrentHubSetpoint().getTurretRadiansFromCenter(),
              state.getCurrentHubSetpoint().getTurretFF());
      case PASS_TRACKING ->
          setPos(
              state.getCurrentPassSetpoint().getTurretRadiansFromCenter(),
              state.getCurrentPassSetpoint().getTurretFF());
      case TUNING -> setPos(tunedSetpoint, 0.0);
      default -> io.stop();
    }

    Logger.recordOutput("Turret/Desired", desiredPos);
  }

  private void setPos(double positionRad, double feedforwardVolts) {
    positionRad =
        MathUtil.clamp(
            positionRad, TurretConstants.kBackwardSoftLimit, TurretConstants.kForwardSoftLimit);
    desiredPos = positionRad;
    Setpoint.motionMagicPosition(positionRad).applyWithFeedforward(io, feedforwardVolts);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public double getDesiredPositionRad() {
    return desiredPos;
  }

  public Rotation2d getRotation() {
    return new Rotation2d(inputs.positionRad).minus(TurretConstants.kTurretAbsEncoderOffset);
  }

  public boolean atSetpoint(double toleranceRad) {
    return Math.abs(inputs.positionRad - desiredPos) < toleranceRad;
  }

  public void setTuningSetpoint(double radians) {
    tunedSetpoint = radians;
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    HUB_TRACKING,
    PASS_TRACKING,
    TUNING
  }
}
