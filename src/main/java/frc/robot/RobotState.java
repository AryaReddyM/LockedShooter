package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionIOSimPhoton;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.shooting.ShooterSetpoint;
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
  private int simFuelCount = 0;

  private static final double kSimLaunchHeightMeters = 0.6;
  private static final double kSimLaunchPeriodSeconds = 0.1;
  private static final int kSimStartingFuel = 20;

  private ShooterSetpoint hubSetpoint =
      new ShooterSetpoint(100.0, Units.degreesToRadians(35), HoodConstants.kHoodG, 0.0, 0.0);
  private ShooterSetpoint passSetpoint =
      new ShooterSetpoint(60.0, Units.degreesToRadians(45), HoodConstants.kHoodG, 0.0, 0.0);

  public RobotState() {
    super("RobotState", State.UNDETERMINED, State.class);

    drive = createDrive();
    shooter =
        new Shooter(
            this, TurretConstants.createIO(), HoodConstants.createIO(), FlywheelConstants.createIO());
    intake = new Intake(IntakeConstants.createExtensionIO(), IntakeConstants.createRollersIO());
    hopper = new Hopper(HopperConstants.createIO());
    kicker = new Kicker(KickerConstants.createIO());
    climb =
        new Climb(ClimbConstants.createIO(), new BeamBreakerIOSim(), new BeamBreakerIOSim());
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

    fuelSim = createFuelSim();

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
        return new VisionIOSimPhoton(drive::getPose, () -> shooter.getTurret().getRotation());
      default:
        return new VisionIO() {};
    }
  }

  private FuelSim createFuelSim() {
    FuelSim sim = new FuelSim();
    if (Constants.currentMode == Constants.Mode.SIM) {
      sim.registerRobot(
          DriveConstants.trackWidth,
          DriveConstants.wheelBase,
          DriveConstants.kBumperHeight,
          drive::getPose,
          () -> ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()));
      sim.spawnStartingFuel();
      sim.start();
      simFuelCount = kSimStartingFuel;
      fuelLaunchTimer.start();
    }
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
    Logger.recordOutput("RobotState/HubRPS", hubSetpoint.getShooterRPS());
    Logger.recordOutput("RobotState/PassRPS", passSetpoint.getShooterRPS());

    if (Constants.currentMode == Constants.Mode.SIM) {
      updateFuelSim();
    }
  }

  private void updateFuelSim() {
    Superstructure.State ssState = superstructure.getState();
    boolean shooting = ssState == Superstructure.State.SHOOTING;
    boolean passing = ssState == Superstructure.State.PASSING;

    if ((shooting || passing) && simFuelCount > 0 && fuelLaunchTimer.hasElapsed(kSimLaunchPeriodSeconds)) {
      ShooterSetpoint sp = shooting ? hubSetpoint : passSetpoint;
      fuelSim.launchFuel(
          MetersPerSecond.of(sp.getShooterRPS()),
          Radians.of(Math.PI / 2.0 - sp.getHoodRadians()),
          Radians.of(sp.getTurretRadiansFromCenter()),
          Meters.of(kSimLaunchHeightMeters));
      simFuelCount--;
      fuelLaunchTimer.reset();
    }

    fuelSim.updateSim();
    Logger.recordOutput("RobotState/SimFuelCount", simFuelCount);
  }

  @Override
  protected void determineSelf() {
    setState(State.ENABLED);
  }

  public ShooterSetpoint getCurrentHubSetpoint() {
    return hubSetpoint;
  }

  public ShooterSetpoint getCurrentPassSetpoint() {
    return passSetpoint;
  }

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
