package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.BeamBreakerIOSim;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionIOSimPhoton;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.logging.ComponentVisualizer;
import frc.robot.util.shooting.ShooterSetpoint;
import frc.robot.util.shooting.ShotCalculator;
import frc.robot.util.sim.FuelSim;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class RobotState extends StateMachine<RobotState.State> {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Hopper hopper;
  private final Kicker kicker;
  private final Climb climb;
  private final Elevator elevator;
  private final VisionSubsystem vision;
  private final Superstructure superstructure;

  private final FuelSim fuelSim;
  private final Timer fuelLaunchTimer = new Timer();
  private final Timer fuelIntakeTimer = new Timer();
  private int simFuelCount = 0;

  /**
   * Height the fuel leaves the shooter at. {@link FuelSim#launchFuel} spawns the ball directly
   * above the robot's origin at this height, and the shot solver aims from the same point, so the
   * two stay consistent.
   */
  private static final double kLaunchHeightMeters = 0.6;

  /** Minimum spacing between shots, which is what the kicker can physically feed. */
  private static final double kLaunchPeriodSeconds = 0.25;

  /**
   * How much fuel the robot can hold. The hopper funnel measures roughly 0.66 x 0.51 x 0.44 m in
   * CAD, which at sensible packing is dozens of 0.15 m balls, so this is not the binding limit in
   * practice; set it to whatever the game actually allows. It used to be 20, which a driver hit
   * within about a second of touching the centre pile, after which the intake silently stopped
   * working and the robot just shoved balls around.
   */
  private static final int kFuelCapacity = 40;

  private static final int kSimStartingFuel = 5;

  /**
   * Minimum spacing between pickups. Without this the fuel simulation swallows every ball inside
   * the intake's box on every physics subtick, so driving through a pile inhales fifteen balls in
   * a second. A real intake feeds them in one at a time.
   */
  private static final double kIntakeIntervalSeconds = 0.15;

  // Bounding box of the intake, robot relative, in meters. It reaches past the bumper so it
  // picks up fuel that the robot collision has already pushed clear of the frame.
  private static final double kIntakeMinX = 0.25;
  private static final double kIntakeMaxX = 0.62;
  private static final double kIntakeHalfWidth = 0.33;

  /** Height a passed ball should arrive at, low enough for a teammate to collect. */
  private static final double kPassTargetHeightMeters = 0.6;

  /**
   * How far short of the funnel centre the fuel already has to be at funnel height. The hub is a
   * solid box up to just under the opening, so a shot that is still climbing when it gets there
   * hits the near wall instead of dropping in.
   */
  private static final double kHubClearanceRadiusMeters =
      VisionConstants.Hub.width / 2.0 + FuelSim.FUEL_RADIUS;

  /**
   * Where the robot appears when the simulation starts. The pose estimator otherwise begins at the
   * field corner, which is inside a wall and out of shooting range. This is a sensible spot in the
   * blue alliance zone, facing the hub.
   */
  private static final Pose2d kSimStartPose = new Pose2d(1.6, 4.02, Rotation2d.kZero);

  private ShooterSetpoint hubSetpoint = ShooterSetpoint.idle();
  private ShooterSetpoint passSetpoint = ShooterSetpoint.idle();

  public RobotState() {
    super("RobotState", State.UNDETERMINED, State.class);

    drive = createDrive();
    shooter =
        new Shooter(
            this,
            TurretConstants.createIO(),
            HoodConstants.createIO(),
            FlywheelConstants.createIO());
    intake = new Intake(IntakeConstants.createExtensionIO(), IntakeConstants.createRollersIO());
    hopper = new Hopper(HopperConstants.createIO());
    kicker = new Kicker(KickerConstants.createIO());
    climb = new Climb(ClimbConstants.createIO(), new BeamBreakerIOSim(), new BeamBreakerIOSim());
    elevator = new Elevator(ElevatorConstants.createIO());
    vision = new VisionSubsystem(createVisionIO(), drive);
    superstructure = new Superstructure(shooter, intake, hopper, kicker, climb);

    addChildSubsystem(drive);
    addChildSubsystem(shooter);
    addChildSubsystem(intake);
    addChildSubsystem(hopper);
    addChildSubsystem(kicker);
    addChildSubsystem(climb);
    addChildSubsystem(elevator);
    addChildSubsystem(vision);
    addChildSubsystem(superstructure);

    if (Constants.currentMode == Constants.Mode.SIM) {
      drive.setPose(kSimStartPose);
    }

    fuelSim = createFuelSim();
    updateShooterSetpoints();

    if (Constants.currentMode == Constants.Mode.SIM) {
      SmartDashboard.putData(
          "Reset Fuel", Commands.runOnce(this::resetFuel).ignoringDisable(true).withName("Reset Fuel"));
      SmartDashboard.putData(
          "Reset Hub Scores",
          Commands.runOnce(this::resetScores).ignoringDisable(true).withName("Reset Hub Scores"));
    }

    addOmniTransitions(State.ENABLED);
  }

  private VisionIO createVisionIO() {
    switch (Constants.currentMode) {
      case REAL:
        return new VisionIOHardwareLimelight(
            drive::getRotation,
            () -> drive.getChassisSpeeds().omegaRadiansPerSecond,
            () -> shooter.getTurret().getRotation(),
            () -> 0.0);
      case SIM:
        // Ground truth, not the estimator's own output; see Drive.getWheelOdometryPose.
        return new VisionIOSimPhoton(
            drive::getWheelOdometryPose, () -> shooter.getTurret().getRotation());
      default:
        return new VisionIO() {};
    }
  }

  private FuelSim createFuelSim() {
    FuelSim sim = new FuelSim();
    if (Constants.currentMode != Constants.Mode.SIM) {
      return sim;
    }

    sim.registerRobot(
        DriveConstants.trackWidth,
        DriveConstants.wheelBase,
        DriveConstants.kBumperHeight,
        drive::getPose,
        this::getFieldRelativeSpeeds);

    // Known issue 5 in CLAUDE.md: the intake was never registered, so fuel could not be
    // picked up in sim. Intaking only works while the rollers are actually running and
    // there is somewhere to put the ball.
    sim.registerIntake(
        kIntakeMinX,
        kIntakeMaxX,
        -kIntakeHalfWidth,
        kIntakeHalfWidth,
        () ->
            intake.getState() == Intake.State.INTAKE
                && simFuelCount < kFuelCapacity
                && fuelIntakeTimer.hasElapsed(kIntakeIntervalSeconds),
        () -> {
          simFuelCount++;
          fuelIntakeTimer.reset();
        });

    // The shot solver models drag, so the sim has to as well or every shot lands long.
    sim.enableAirResistance();
    sim.spawnStartingFuel();
    sim.start();
    simFuelCount = kSimStartingFuel;
    fuelLaunchTimer.start();
    fuelIntakeTimer.start();
    return sim;
  }

  private static Drive createDrive() {
    switch (Constants.currentMode) {
      case REAL:
        boolean primary = Constants.robot == Constants.RobotType.PRIMARY;
        return new Drive(
            new GyroIOPigeon2(),
            primary ? new ModuleIOTalonFX(0) : new ModuleIOSpark(0),
            primary ? new ModuleIOTalonFX(1) : new ModuleIOSpark(1),
            primary ? new ModuleIOTalonFX(2) : new ModuleIOSpark(2),
            primary ? new ModuleIOTalonFX(3) : new ModuleIOSpark(3));
      case SIM:
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
      default:
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }
  }

  @Override
  protected void update() {
    updateShooterSetpoints();
    ComponentVisualizer.log(this);

    if (Constants.currentMode == Constants.Mode.SIM) {
      updateFuelSim();
    }
  }

  /** Recomputes both shots for where the robot is right now. */
  private void updateShooterSetpoints() {
    Pose2d pose = getLatestFieldToRobot();
    ChassisSpeeds speeds = getFieldRelativeSpeeds();
    Translation3d launchPoint =
        new Translation3d(pose.getX(), pose.getY(), kLaunchHeightMeters);

    boolean drag = fuelSim.isAirResistanceEnabled();
    hubSetpoint =
        ShotCalculator.solve(
            launchPoint, pose, speeds, getHubTarget(), kHubClearanceRadiusMeters, drag);
    passSetpoint =
        ShotCalculator.solve(launchPoint, pose, speeds, getPassTarget(), 0.0, drag);

    Logger.recordOutput("RobotState/IsRedAlliance", isRedAlliance());
    Logger.recordOutput("RobotState/HubTarget", getHubTarget());
    Logger.recordOutput("RobotState/Hub/SurfaceSpeed", hubSetpoint.getFlywheelSurfaceSpeed());
    Logger.recordOutput("RobotState/Hub/HoodRadians", hubSetpoint.getHoodRadians());
    Logger.recordOutput("RobotState/Hub/TurretRadians", hubSetpoint.getTurretRadiansFromCenter());
    Logger.recordOutput("RobotState/Hub/Achievable", hubSetpoint.isAchievable());
    Logger.recordOutput("RobotState/Pass/SurfaceSpeed", passSetpoint.getFlywheelSurfaceSpeed());
    Logger.recordOutput("RobotState/Pass/Achievable", passSetpoint.isAchievable());
  }

  private void updateFuelSim() {
    Superstructure.State ssState = superstructure.getState();
    boolean shooting = ssState == Superstructure.State.SHOOTING;
    boolean passing = ssState == Superstructure.State.PASSING;
    ShooterSetpoint active = shooting ? hubSetpoint : passSetpoint;

    // Every condition that has to hold before a ball leaves the shooter, kept as separate
    // named values so each one can be logged. When nothing comes out of the shooter, the
    // question is always "which of these is false", and a single combined boolean cannot
    // answer it.
    boolean inShootingState = shooting || passing;
    boolean hasFuel = simFuelCount > 0;
    boolean shotAchievable = active.isAchievable();
    boolean mechanismsReady = shooter.readyToShoot();
    boolean feedIntervalElapsed = fuelLaunchTimer.hasElapsed(kLaunchPeriodSeconds);

    boolean canLaunch =
        inShootingState && hasFuel && shotAchievable && mechanismsReady && feedIntervalElapsed;

    Logger.recordOutput("RobotState/Sim/InShootingState", inShootingState);
    Logger.recordOutput("RobotState/Sim/HasFuel", hasFuel);
    Logger.recordOutput("RobotState/Sim/ShotAchievable", shotAchievable);
    Logger.recordOutput("RobotState/Sim/MechanismsReady", mechanismsReady);
    Logger.recordOutput("RobotState/Sim/FeedIntervalElapsed", feedIntervalElapsed);
    Logger.recordOutput("RobotState/Sim/CanLaunch", canLaunch);

    if (canLaunch) {
      // FuelSim wants the launch angle measured up from horizontal, which is exactly how
      // the hood angle is defined, and the fuel's own speed rather than the flywheel's
      // surface speed. Passing the flywheel setpoint straight through as m/s was known
      // issue 2 in CLAUDE.md and made every shot fly roughly twice as far as intended.
      fuelSim.launchFuel(
          MetersPerSecond.of(active.getBallExitSpeed()),
          Radians.of(active.getHoodRadians()),
          Radians.of(active.getTurretRadiansFromCenter()),
          Meters.of(kLaunchHeightMeters));
      simFuelCount--;
      fuelLaunchTimer.reset();
    }

    fuelSim.updateSim();

    Logger.recordOutput("RobotState/SimFuelCount", simFuelCount);
    // Worth watching: once this latches true the intake stops collecting, and without it
    // the sim just looks broken.
    Logger.recordOutput("RobotState/FuelAtCapacity", simFuelCount >= kFuelCapacity);
    // Balls loose on the field, as opposed to SimFuelCount which is what the robot carries.
    // Watching the two side by side shows whether the intake is collecting, the shooter is
    // firing, or neither.
    Logger.recordOutput("RobotState/Sim/FuelOnField", fuelSim.getFuelCount());
    Logger.recordOutput("RobotState/ReadyToShoot", mechanismsReady);
    Logger.recordOutput("RobotState/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("RobotState/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
  }

  @Override
  protected void determineSelf() {
    setState(State.ENABLED);
  }

  @Override
  public void onAutonomousStart() {
    // Entering autonomous is the start of a match, so put the field back how it started.
    resetFuel();
  }

  /**
   * Puts every piece of fuel back where it started, empties the robot down to its preload, and
   * zeroes both hubs' scores.
   *
   * <p>Bound to the "Reset Fuel" dashboard button, and run automatically when autonomous starts.
   * Without it the only way to get the field back after driving through the pile was to restart
   * the simulation.
   */
  public void resetFuel() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return;
    }
    fuelSim.clearFuel();
    fuelSim.spawnStartingFuel();
    simFuelCount = kSimStartingFuel;
    fuelIntakeTimer.reset();
    fuelLaunchTimer.reset();
    resetScores();
  }

  /** Zeroes both hubs' scores without touching the fuel on the field. */
  public void resetScores() {
    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
  }

  // --- Field state -------------------------------------------------------------------

  /** Best current estimate of where the robot is on the field. */
  public Pose2d getLatestFieldToRobot() {
    return drive.getPose();
  }

  /** Robot velocity in field coordinates. */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Centre of the funnel opening on our own hub, which is the point a shot has to drop through.
   *
   * <p>The hub poses in {@link VisionConstants} sit at the <i>base</i> of the funnel (56.4 in); the
   * fuel counts as scored when it crosses the rim at the top of the hub (72 in), so that is what
   * the shot has to be aimed at.
   */
  public Translation3d getHubTarget() {
    Translation3d hub = isRedAlliance() ? VisionConstants.kRedHubPose : VisionConstants.kBlueHubPose;
    return new Translation3d(hub.getX(), hub.getY(), VisionConstants.Hub.height);
  }

  /**
   * Where a pass should land: over our own hub but at collecting height rather than through the
   * funnel, so a teammate can pick it up.
   */
  public Translation3d getPassTarget() {
    Translation3d hub = isRedAlliance() ? VisionConstants.kRedHubPose : VisionConstants.kBlueHubPose;
    return new Translation3d(hub.getX(), hub.getY(), kPassTargetHeightMeters);
  }

  public ShooterSetpoint getCurrentHubSetpoint() {
    return hubSetpoint;
  }

  public ShooterSetpoint getCurrentPassSetpoint() {
    return passSetpoint;
  }

  // --- Subsystem access --------------------------------------------------------------

  public Drive getDrive() {
    return drive;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Intake getIntake() {
    return intake;
  }

  public Hopper getHopper() {
    return hopper;
  }

  public Kicker getKicker() {
    return kicker;
  }

  public Climb getClimb() {
    return climb;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public VisionSubsystem getVision() {
    return vision;
  }

  public FuelSim getFuelSim() {
    return fuelSim;
  }

  public int getSimFuelCount() {
    return simFuelCount;
  }

  public enum State {
    UNDETERMINED,
    ENABLED
  }
}
