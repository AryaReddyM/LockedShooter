package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.state.StateMachine;

public class Shooter extends StateMachine<Shooter.State> {
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  public Shooter(RobotState state, MotorIO turretIO, MotorIO hoodIO, MotorIO flywheelIO) {
    super("Shooter", State.UNDETERMINED, State.class);

    turret = new Turret(turretIO, state);
    hood = new Hood(hoodIO, state);
    flywheel = new Flywheel(flywheelIO, state);

    addChildSubsystem(turret);
    addChildSubsystem(hood);
    addChildSubsystem(flywheel);

    addOmniTransitions(
        State.IDLE,
        State.HUB_TRACKING,
        State.PASS_TRACKING,
        State.SHOOTING,
        State.PASSING,
        State.OUTTAKE,
        State.TUNING);

    registerStateCommand(State.IDLE, cascade(Turret.State.IDLE, Hood.State.IDLE, Flywheel.State.IDLE));
    // Tracking spins the flywheel all the way up rather than idling it: readyToShoot()
    // includes the flywheel, so anything waiting on it before feeding would otherwise
    // wait forever, and pre-spinning is what makes the shot go off the moment the
    // driver asks for it.
    registerStateCommand(
        State.HUB_TRACKING,
        cascade(Turret.State.HUB_TRACKING, Hood.State.HUB_TRACKING, Flywheel.State.SHOOT));
    registerStateCommand(
        State.PASS_TRACKING,
        cascade(Turret.State.PASS_TRACKING, Hood.State.PASS_TRACKING, Flywheel.State.PASS));
    registerStateCommand(
        State.SHOOTING,
        cascade(Turret.State.HUB_TRACKING, Hood.State.HUB_TRACKING, Flywheel.State.SHOOT));
    registerStateCommand(
        State.PASSING,
        cascade(Turret.State.PASS_TRACKING, Hood.State.PASS_TRACKING, Flywheel.State.PASS));
    registerStateCommand(
        State.OUTTAKE, cascade(Turret.State.IDLE, Hood.State.IDLE, Flywheel.State.IDLE));
    registerStateCommand(
        State.TUNING, cascade(Turret.State.TUNING, Hood.State.TUNING, Flywheel.State.TUNING));
  }

  private InstantCommand cascade(
      Turret.State turretState, Hood.State hoodState, Flywheel.State flywheelState) {
    return new InstantCommand(
        () -> {
          turret.requestTransition(turretState);
          hood.requestTransition(hoodState);
          flywheel.requestTransition(flywheelState);
        });
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public boolean readyToShoot(double turretTolRad, double hoodTolRad) {
    return flywheel.isReady()
        && turret.atSetpoint(turretTolRad)
        && hood.atSetpoint(hoodTolRad);
  }

  /** Ready to shoot using each mechanism's own configured tolerance. */
  public boolean readyToShoot() {
    return readyToShoot(
        Units.degreesToRadians(TurretConstants.kReadyToleranceDegrees),
        HoodConstants.kReadyToleranceRadians);
  }

  public Turret getTurret() {
    return turret;
  }

  public Hood getHood() {
    return hood;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    HUB_TRACKING,
    PASS_TRACKING,
    SHOOTING,
    PASSING,
    OUTTAKE,
    TUNING
  }
}
