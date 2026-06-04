package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.climb.BeamBreakerIO;
import frc.robot.subsystems.climb.BeamBreakerSim;
import frc.robot.subsystems.climb.BeamBreakerTOF;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSpark;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOSpark;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionIOSimPhoton;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.CustomAutoBuilder;
import frc.robot.util.DynamicPathGenerator;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.FuelSim;
import frc.robot.util.MathHelpers;
import frc.robot.util.ShooterSetpoint;
import frc.robot.util.SimulatedRobotState;
import frc.robot.util.TrenchZone;
import frc.robot.util.state.StateMachine;

public class RobotState extends StateMachine<RobotState.State> {
    public final static double LOOKBACK_TIME = 1.0;
    public final static AtomicBoolean hubActivated = new AtomicBoolean();

    private Pose2d lookAtPose;
    private final SimulatedRobotState simulatedRobotState = Robot.isSimulation() ? new SimulatedRobotState(this) : null;

    private Drive drive;
    private VisionSubsystem vision;
    private Shooter shooter;
    private Climb climb;
    private Intake intake;
    private Hopper hopper;
    private Kicker kicker;

    private Command autoCommand = null;
    CustomAutoBuilder customAutoBuilder;

    private Supplier<ShooterSetpoint> hubSupplier;
    private Supplier<ShooterSetpoint> passSupplier;

    private FuelSim fuelSim = new FuelSim();
    private double simFuelCount = 8;

    private boolean climbZeroed = false;
    private boolean visionDisabled = false;

    private CommandXboxController controller = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    private final LoggedDashboardChooser<Command> autoChooser;

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;

    // kinematic frame
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot = ConcurrentTimeInterpolatableBuffer
            .createBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Rotation2d> robotToTurret = ConcurrentTimeInterpolatableBuffer
            .createBuffer(LOOKBACK_TIME);
    private static final Transform2d TURRET_TO_CAMERA = new Transform2d(VisionConstants.kTurretToCameraX,
            VisionConstants.kTurretToCameraY,
            MathHelpers.kRotation2dZero);

    private static final Transform2d ROBOT_TO_CAMERA_B = new Transform2d();
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());

    private double lastUsedMegatagTimestamp = 0;
    private ConcurrentTimeInterpolatableBuffer<Double> turretAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> turretPositionRadians = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveRollRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> accelX = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> accelY = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private final AtomicBoolean enablePathCancel = new AtomicBoolean(false);

    private double autoStartTime;

    public RobotState() {
        super("RobotState", State.UNDETERMINED, State.class);

        // vision initialization
        {
            clearBuffers();

            visionEstimateConsumer = new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate estimate) {
                    if (visionDisabled) {
                        return;
                    }
                    if (Constants.currentMode != Constants.Mode.REAL) {
                        return;
                    }
                    drive.addVisionMeasurement(estimate.getVisionRobotPoseMeters(), estimate.getTimestampSeconds(),
                            estimate.getVisionMeasurementStdDevs());
                }
            };

            switch (Constants.currentMode) {
                case REAL:
                    vision = new VisionSubsystem(new VisionIOHardwareLimelight(this), this);
                    break;
                default:
                    vision = new VisionSubsystem(new VisionIOSimPhoton(this, simulatedRobotState), this);
                    break;
            }

            Elastic.sendNotification(
                    new Notification().withTitle("Vision Subsystem").withDescription("Vision Started"));
        }

        // drive initialization
        {
            lookAtPose = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                    ? new Pose2d(VisionConstants.FieldConstants.HUB_BLUE.toTranslation2d(), Rotation2d.kZero)
                    : new Pose2d(VisionConstants.FieldConstants.HUB_RED.toTranslation2d(), Rotation2d.kZero);

            switch (Constants.currentMode) {
                case REAL:
                    drive = new Drive(
                            new GyroIOPigeon2(),
                            new ModuleIOSpark(0),
                            new ModuleIOSpark(1),
                            new ModuleIOSpark(2),
                            new ModuleIOSpark(3),
                            this);
                    break;
                case SIM:
                    drive = new Drive(
                            new GyroIO() {
                            },
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            this);

                    fuelSim.spawnStartingFuel();

                    fuelSim.start();
                    fuelSim.enableAirResistance();

                    SmartDashboard.putData(Commands.runOnce(() -> {
                        fuelSim.clearFuel();
                        fuelSim.spawnStartingFuel();
                    })
                            .withName("Reset Fuel")
                            .ignoringDisable(true));

                    fuelSim.registerRobot(
                            Meter.of(DriveConstants.trackWidth),
                            Meter.of(DriveConstants.wheelBase),
                            Meter.of(DriveConstants.kBumperHeight),
                            () -> {
                                Pose2d drivePose = getLatestFieldToRobot().getValue();
                                return drivePose.transformBy(
                                        new Transform2d(
                                                VisionConstants.kTurretToRobotCenter.getTranslation().toTranslation2d(),
                                                Rotation2d.kZero));
                            },
                            this::getLatestDesiredFieldRelativeChassisSpeed);

                    fuelSim.registerIntake(
                            Meters.of(DriveConstants.trackWidth +
                                    Units.inchesToMeters(6)).div(2).in(Meters),
                            Meters.of(DriveConstants.trackWidth +
                                    Units.inchesToMeters(6)).div(2).plus(Inches.of(8.5))
                                    .in(Meters),
                            -Meters.of(DriveConstants.trackWidth +
                                    Units.inchesToMeters(6)).div(2).in(Meters),
                            Meters.of(DriveConstants.trackWidth +
                                    Units.inchesToMeters(6)).div(2).in(Meters),
                            () -> intake.getState() == Intake.State.INTAKE,
                            () -> {
                                simFuelCount++;
                            });
                    break;
                case REPLAY:
                    drive = new Drive(
                            new GyroIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            }, this);
                    break;
            }

            Elastic.sendNotification(new Notification().withTitle("Drive Subsystem").withDescription("Drive Started"));
        }

        { // shooter
            hubSupplier = ShooterSetpoint.speakerSetpointSupplier(this);
            passSupplier = ShooterSetpoint.passSetpointSupplier(this);

            switch (Constants.currentMode) {
                case REAL:
                    shooter = new Shooter(
                            this,
                            new TurretIOSpark(),
                            new HoodIOSpark(),
                            new FlywheelIOSpark());
                    break;
                case SIM:
                    shooter = new Shooter(
                            this,
                            new TurretIOSim(),
                            new HoodIOSim(),
                            new FlywheelIOSim());
                    break;
                case REPLAY:
                    shooter = new Shooter(
                            this,
                            new TurretIO() {
                            },
                            new HoodIO() {
                            },
                            new FlywheelIO() {
                            });
                    break;
            }
        }

        { // climb
            switch (Constants.currentMode) {
                case REAL:
                    climb = new Climb(
                            new ClimbIOSpark(),
                            new BeamBreakerTOF(1),
                            new BeamBreakerTOF(2),
                            this);
                    break;
                case SIM:
                    climb = new Climb(
                            new ClimbIOSim(),
                            new BeamBreakerSim(1, this),
                            new BeamBreakerSim(2, this),
                            this);
                    break;
                case REPLAY:
                    climb = new Climb(
                            new ClimbIO() {
                            },
                            new BeamBreakerIO() {
                            },
                            new BeamBreakerIO() {
                            },
                            this);
                    break;
            }
        }

        { // hopper
            switch (Constants.currentMode) {
                case REAL:
                    hopper = new Hopper(
                            new HopperIOSpark(),
                            this);
                    break;
                case SIM:
                    hopper = new Hopper(
                            new HopperIOSim(),
                            this);
                    break;
                case REPLAY:
                    hopper = new Hopper(
                            new HopperIO() {
                            },
                            this);
                    break;
            }
        }

        { // intake
            switch (Constants.currentMode) {
                case REAL:
                    intake = new Intake(
                            new IntakeIOSpark(),
                            this);
                    break;
                case SIM:
                    intake = new Intake(
                            new IntakeIOSim(),
                            this);
                    break;
                case REPLAY:
                    intake = new Intake(
                            new IntakeIO() {
                            },
                            this);
                    break;
            }
        }

        { // kicker
            switch (Constants.currentMode) {
                case REAL:
                    kicker = new Kicker(
                            new KickerIOSpark(),
                            this);
                    break;
                case SIM:
                    kicker = new Kicker(
                            new KickerIOSim(),
                            this);
                    break;
                case REPLAY:
                    kicker = new Kicker(
                            new KickerIO() {
                            },
                            this);
                    break;
            }
        }

        // auto setup
        {
            customAutoBuilder = new CustomAutoBuilder(this);
            autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
            setupDriveDiagnosisAutos();
        }

        CameraServer.startAutomaticCapture();

        setupControllerBindings();
        setupNotis();

        registerStateTransitions();
        registerStateCommands();

        addChildSubsystem(vision);
        addChildSubsystem(drive);
        addChildSubsystem(shooter);
        addChildSubsystem(climb);
        addChildSubsystem(hopper);
        addChildSubsystem(intake);
        addChildSubsystem(kicker);
        enable();

        Logger.recordOutput("Bumper/Pose", new Pose3d());

        try {
            DynamicPathGenerator.warmupInit();
        } catch (Exception e) {
            Elastic.sendNotification(new Notification().withTitle("Warmup Command").withLevel(NotificationLevel.ERROR)
                    .withDescription("Failed to warmup commands"));
        }
    }

    private void setupDriveDiagnosisAutos() {

        autoChooser.addOption("Center Only Starting 8 (GAME)",
                AutoCommands.getAutoByName(this, "Center Only Starting 8 (GAME)").get().getCommand(this));

        autoChooser.addOption("Center Only Starting 8 Climb (GAME)",
                AutoCommands.getAutoByName(this, "Center Only Starting 8 Climb (GAME)").get().getCommand(this));

        autoChooser.addOption("Depot Only Starting 8 (GAME)",
                AutoCommands.getAutoByName(this, "Depot Only Starting 8 (GAME)").get().getCommand(this));

        autoChooser.addOption("Depot Only Starting 8 Climb (GAME)",
                AutoCommands.getAutoByName(this, "Depot Only Starting 8 Climb (GAME)").get().getCommand(this));

        autoChooser.addOption("Depot Side To Depot (GAME)",
                AutoCommands.getAutoByName(this, "Depot Side To Depot (GAME)").get().getCommand(this));

        autoChooser.addOption("Depot Side To Depot Climb (GAME)",
                AutoCommands.getAutoByName(this, "Depot Side To Depot Climb (GAME)").get().getCommand(this));

        autoChooser.addOption("Depot Side To Depot End at Mid (GAME)",
                AutoCommands.getAutoByName(this, "Depot Side To Depot End at Mid (GAME)").get().getCommand(this));

        autoChooser.addOption("HP Only Starting 8 (GAME)",
                AutoCommands.getAutoByName(this, "HP Only Starting 8 (GAME)").get().getCommand(this));

        autoChooser.addOption("HP Only Starting 8 Climb (GAME)",
                AutoCommands.getAutoByName(this, "HP Only Starting 8 Climb (GAME)").get().getCommand(this));

        autoChooser.addOption("HP Side To HP (GAME)",
                AutoCommands.getAutoByName(this, "HP Side To HP (GAME)").get().getCommand(this));

        autoChooser.addOption("HP Side To HP Climb (GAME)",
                AutoCommands.getAutoByName(this, "HP Side To HP Climb (GAME)").get().getCommand(this));

        autoChooser.addOption("HP Side To HP End at Mid (GAME)",
                AutoCommands.getAutoByName(this, "HP Side To HP End at Mid (GAME)").get().getCommand(this));

        autoChooser.addOption("Custom Auto Builder", customAutoBuilder.getCommand(this));

        autoChooser.addOption("Depot Side Depot Mid Half Sweep (GAME)",
                AutoCommands.getAutoByName(this, "Depot Side Depot Mid Half Sweep (GAME)").get().getCommand(this));
        autoChooser.addOption("Depot Side Quick Shoot Climb",
                AutoCommands.getAutoByName(this, "Depot Side Quick Shoot Climb (GAME)").get().getCommand(this));
        autoChooser.addOption("HP Side Quick Shoot Climb",
                AutoCommands.getAutoByName(this, "HP Side Quick Shoot Climb (GAME)").get().getCommand(this));
        autoChooser.addOption("Depot Side Circut Shoot (GAME)",
                AutoCommands.getAutoByName(this, "Depot Side Circut Shoot (GAME)").get().getCommand(this));
    }

    public void clearBuffers() {
        fieldToRobot.clear();
        robotToTurret.clear();
        turretAngularVelocity.clear();
        driveYawAngularVelocity.clear();
        turretPositionRadians.clear();

        fieldToRobot.addSample(0.0, MathHelpers.kPose2dZero);
        robotToTurret.addSample(0.0, MathHelpers.kRotation2dZero);
        turretAngularVelocity.addSample(0.0, 0.0);
        driveYawAngularVelocity.addSample(0.0, 0.0);
        turretPositionRadians.addSample(0.0, 0.0);
    }

    public void resetBuffersToPose(Pose2d pose) {
        double now = Timer.getFPGATimestamp();

        fieldToRobot.clear();
        fieldToRobot.addSample(now, pose);
    }

    private void setupNotis() {
        new Trigger(() -> hubActivated.get())
                .onChange(
                        new StartEndCommand(
                                () -> {
                                    controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                                    operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                                },
                                () -> {
                                    controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                }).withTimeout(0.5));
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.SOFT_STOP, State.TRAVERSING, State.AUTO, State.SHOOTING, State.SHOOTING_INTAKING,
                State.CLIMBING, State.INTAKING, State.PASSING, State.PASSING_INTAKING);
    }

    private void registerStateCommands() {
        registerStateCommand(State.SOFT_STOP, new ParallelCommandGroup(
                drive.transitionCommand(Drive.State.IDLE)));

        registerStateCommand(State.TRAVERSING, new ParallelCommandGroup(
                drive.transitionCommand(Drive.State.TRAVERSING)));

        registerStateCommand(State.INTAKING, new ParallelCommandGroup());

        registerStateCommand(State.SHOOTING, new ParallelCommandGroup());

        registerStateCommand(State.PASSING, new ParallelCommandGroup());

        registerStateCommand(State.SHOOTING_INTAKING, new ParallelCommandGroup());

        registerStateCommand(State.PASSING_INTAKING, new ParallelCommandGroup());

        registerStateCommand(State.CLIMBING, new ParallelCommandGroup());

        registerStateCommand(State.AUTO, new ParallelCommandGroup(
                drive.transitionCommand(Drive.State.TRAVERSING)));
    }

    private void setupControllerBindings() {

        // only works at home, cannot reset pose in a match
        if (DriverStation.getMatchType().equals(MatchType.None)) {
            controller
            .povDown()
            .onTrue(
            Commands.runOnce(
            () -> drive.setPose(
            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
            drive)
            .ignoringDisable(true));
        }

        // driver 1 controller
        {
            controller
                    .leftTrigger(0.5)
                    .onTrue(intake.transitionCommand(Intake.State.INTAKE))
                    .onFalse(intake.transitionCommand(Intake.State.IDLE));

            controller
                    .leftBumper()
                    .onTrue(intake.transitionCommand(Intake.State.STOW));

            controller
                    .rightTrigger(0.5)
                    .onTrue(ActionCommands.shootOrPassBasedOnPos(this))
                    .onFalse(ActionCommands.trackBasedOnPos(this));

            controller.rightBumper().onTrue(drive.transitionCommand(Drive.State.SLOW))
                    .onFalse(drive.transitionCommand(Drive.State.TRAVERSING));

            controller
                    .y()
                    .whileTrue(ActionCommands.autoClimb(this))
                    .onFalse(climb.transitionCommand(Climb.State.STOW));

            controller
                    .a()
                    .onTrue(new InstantCommand(() -> {
                        shooter.requestTransition(Shooter.State.OUTTAKE);
                        intake.requestTransition(Intake.State.OUTAKE);
                    }))
                    .onFalse(new InstantCommand(() -> {
                        hopper.requestTransition(Hopper.State.IDLE);
                        kicker.requestTransition(Kicker.State.IDLE);
                        ActionCommands.trackBasedOnPos(this);
                        intake.requestTransition(Intake.State.IDLE);
                    }));

            controller
                    .x()
                    .whileTrue(ActionCommands.goToFixedPosAndShoot(this))
                    .onFalse(new InstantCommand(() -> {
                        shooter.getFlywheel().setOverride(null);
                        shooter.getHood().setOverride(null);
                        shooter.getTurret().setOverride(null);
                    }));

            controller
                    .b()
                    .whileTrue(ActionCommands.shakeIntake(this));
        }

        // driver two
        {
            operatorController
                    .leftStick()
                    .onTrue(new InstantCommand(() -> {
                        if (climb != null)
                            climb.setOverride(null);
                        if (hopper != null)
                            hopper.setOverride(null);
                        if (kicker != null)
                            kicker.setOverride(null);
                        if (intake != null)
                            intake.setOverride(null);
                        if (shooter != null) {
                            if (shooter.getFlywheel() != null)
                                shooter.getFlywheel().setOverride(null);
                            if (shooter.getHood() != null)
                                shooter.getHood().setOverride(null);
                            if (shooter.getTurret() != null)
                                shooter.getTurret().setOverride(null);
                        }
                    }));

            operatorController
                    .rightStick()
                    .onTrue(new InstantCommand(() -> {
                        if (operatorController.y().getAsBoolean() && climb != null) {
                            climb.setOverride(null);
                        } else if (operatorController.x().getAsBoolean()) {
                            if (hopper != null)
                                hopper.setOverride(null);
                            if (kicker != null)
                                kicker.setOverride(null);
                        } else if (operatorController.b().getAsBoolean() && intake != null) {
                            intake.setOverride(null);
                        } else if (operatorController.a().getAsBoolean() && shooter != null) {
                            if (shooter.getFlywheel() != null)
                                shooter.getFlywheel().setOverride(null);
                            if (shooter.getHood() != null)
                                shooter.getHood().setOverride(null);
                            if (shooter.getTurret() != null)
                                shooter.getTurret().setOverride(null);
                        }
                    }));

            operatorController
                    .leftTrigger(0.5)
                    .onTrue(new InstantCommand(() -> {
                        if (operatorController.y().getAsBoolean() && climb != null) {
                            climb.zero();
                            climb.setOverride((a) -> {
                            });
                        } else if (operatorController.x().getAsBoolean()) {
                            if (hopper != null)
                                hopper.setOverride((a) -> hopper.stop());
                            if (kicker != null)
                                kicker.setOverride((a) -> kicker.stop());
                        } else if (operatorController.b().getAsBoolean() && intake != null) {
                            intake.setOverride((a) -> intake.intakeRoll());
                        } else if (operatorController.a().getAsBoolean() && shooter != null) {
                            var setpoint = getCurrentHubSetpoint();
                            if (shooter.getFlywheel() != null)
                                shooter.getFlywheel().setOverride(() -> setpoint.getShooterRPS());
                            if (shooter.getHood() != null)
                                shooter.getHood().setOverride((a) -> shooter.getHood().setPos(setpoint.getHoodRadians(),
                                        setpoint.getHoodFF()));
                            if (shooter.getTurret() != null)
                                shooter.getTurret().setOverride((a) -> shooter.getTurret()
                                        .setPos(setpoint.getTurretRadiansFromCenter(), setpoint.getTurretFF()));
                        }
                    }));

            operatorController
                    .rightTrigger(0.5)
                    .onTrue(new InstantCommand(() -> {
                        if (operatorController.y().getAsBoolean() && climb != null) {
                            climb.setOverride((a) -> climb.down());
                        } else if (operatorController.x().getAsBoolean()) {
                            if (hopper != null)
                                hopper.setOverride((a) -> hopper.stop());
                            if (kicker != null)
                                kicker.setOverride((a) -> kicker.stop());
                        } else if (operatorController.b().getAsBoolean() && intake != null) {
                            intake.setOverride((a) -> intake.outakeRoll());
                        } else if (operatorController.a().getAsBoolean() && shooter != null) {
                            var setpoint = getCurrentPassSetpoint();
                            if (shooter.getFlywheel() != null)
                                shooter.getFlywheel().setOverride(() -> setpoint.getShooterRPS());
                            if (shooter.getHood() != null)
                                shooter.getHood().setOverride((a) -> shooter.getHood().setPos(setpoint.getHoodRadians(),
                                        setpoint.getHoodFF()));
                            if (shooter.getTurret() != null)
                                shooter.getTurret().setOverride((a) -> shooter.getTurret()
                                        .setPos(setpoint.getTurretRadiansFromCenter(), setpoint.getTurretFF()));
                        }
                    }));

            operatorController
                    .leftBumper()
                    .onTrue(new InstantCommand(() -> {
                        if (operatorController.y().getAsBoolean() && climb != null) {
                            climb.setOverride((a) -> climb.up());
                        } else if (operatorController.x().getAsBoolean()) {
                            if (hopper != null)
                                hopper.setOverride((a) -> hopper.shoot());
                            if (kicker != null)
                                kicker.setOverride((a) -> kicker.shoot());
                        } else if (operatorController.b().getAsBoolean() && intake != null) {
                            intake.setOverride((a) -> intake.stow());
                        } else if (operatorController.a().getAsBoolean() && shooter != null) {
                            var setpoint = getCurrentHubSetpoint();
                            if (shooter.getFlywheel() != null)
                                shooter.getFlywheel().setOverride(() -> 0.0);
                            if (shooter.getHood() != null)
                                shooter.getHood().setOverride((a) -> shooter.getHood().setPos(setpoint.getHoodRadians(),
                                        setpoint.getHoodFF()));
                            if (shooter.getTurret() != null)
                                shooter.getTurret().setOverride((a) -> shooter.getTurret()
                                        .setPos(setpoint.getTurretRadiansFromCenter(), setpoint.getTurretFF()));
                        }
                    }));

            operatorController
                    .rightBumper()
                    .onTrue(new InstantCommand(() -> {
                        if (operatorController.y().getAsBoolean() && climb != null) {
                            climb.setOverride((a) -> climb.stow());
                        } else if (operatorController.x().getAsBoolean()) {
                            if (hopper != null)
                                hopper.setOverride((a) -> hopper.outake());
                            if (kicker != null)
                                kicker.setOverride((a) -> kicker.outtake());
                        } else if (operatorController.b().getAsBoolean() && intake != null) {
                            intake.setOverride((a) -> intake.intake());
                        } else if (operatorController.a().getAsBoolean() && shooter != null) {
                            var setpoint = getCurrentPassSetpoint();
                            if (shooter.getFlywheel() != null)
                                shooter.getFlywheel().setOverride(() -> 0.0);
                            if (shooter.getHood() != null)
                                shooter.getHood().setOverride((a) -> shooter.getHood().setPos(setpoint.getHoodRadians(),
                                        setpoint.getHoodFF()));
                            if (shooter.getTurret() != null)
                                shooter.getTurret().setOverride((a) -> shooter.getTurret()
                                        .setPos(setpoint.getTurretRadiansFromCenter(), setpoint.getTurretFF()));
                        }
                    }));
        }
    }

    public Drive getDrive() {
        return drive;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public VisionSubsystem getVision() {
        return vision;
    }

    public Climb getClimb() {
        return climb;
    }

    public Hopper getHopper() {
        return hopper;
    }

    public Intake getIntake() {
        return intake;
    }

    public Kicker getKicker() {
        return kicker;
    }

    public CommandXboxController getController() {
        return controller;
    }

    public ShooterSetpoint getCurrentHubSetpoint() {
        return hubSupplier.get();
    }

    public ShooterSetpoint getCurrentPassSetpoint() {
        return passSupplier.get();
    }

    public Pose2d getDriveAnglePos() {
        return lookAtPose;
    }

    public void setDriveAngle(Pose2d newPose) {
        lookAtPose = newPose;
    }

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel.set(true);
    }

    public void disablePathCancel() {
        enablePathCancel.set(false);
    }

    public boolean getPathCancel() {
        return enablePathCancel.get();
    }

    public double getSimFuelCount() {
        return simFuelCount;
    }

    public FuelSim getFuelSim() {
        return fuelSim;
    }

    public void setSimFuelCount(double val) {
        simFuelCount = val;
    }

    public CustomAutoBuilder getCustomAutoBuilder() {
        return customAutoBuilder;
    }

    public SimulatedRobotState getSimRobot() {
        return simulatedRobotState;
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void addDriveMotionMeasurements(double timestamp,
            double angularRollRadsPerS,
            double angularPitchRadsPerS,
            double angularYawRadsPerS,
            double pitchRads,
            double rollRads,
            double accelX,
            double accelY,
            ChassisSpeeds desiredFieldRelativeSpeeds,
            ChassisSpeeds measuredSpeeds,
            ChassisSpeeds measuredFieldRelativeSpeeds,
            ChassisSpeeds fusedFieldRelativeSpeeds) {
        this.driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.drivePitchRads.addSample(timestamp, pitchRads);
        this.driveRollRads.addSample(timestamp, rollRads);
        this.accelY.addSample(timestamp, accelY);
        this.accelX.addSample(timestamp, accelX);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = maybeFieldToRobot == null ? MathHelpers.kPose2dZero : maybeFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot
                .exp(new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
    }

    public void addTurretUpdates(double timestamp,
            Rotation2d turretRotation,
            double turretRadians,
            double angularYawRadsPerS) {
        robotToTurret.addSample(timestamp, turretRotation);
        this.turretAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.turretPositionRadians.addSample(timestamp, turretRadians);
    }

    public double getLatestTurretPositionRadians() {
        return this.turretPositionRadians.getInternalBuffer().lastEntry().getValue();
    }

    public double getLatestTurretAngularVelocity() {
        return this.turretAngularVelocity.getInternalBuffer().lastEntry().getValue();
    }

    public Transform2d getTurretToCamera(boolean isTurretCamera) {
        return isTurretCamera ? TURRET_TO_CAMERA : ROBOT_TO_CAMERA_B;
    }

    public Optional<Rotation2d> getRobotToTurret(double timestamp) {
        return robotToTurret.getSample(timestamp);
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public Map.Entry<Double, Rotation2d> getLatestRobotToTurret() {
        return robotToTurret.getLatest();
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }

    public Optional<Double> getTurretAngularVelocity(double timestamp) {
        return turretAngularVelocity.getSample(timestamp);
    }

    private Optional<Double> getMaxAbsValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime,
            double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty())
            return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get()))
            return max;
        else
            return min;
    }

    public Optional<Double> getMaxAbsTurretYawAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(turretAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        if (Robot.isReal())
            return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
        return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    }

    public Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        lastUsedMegatagTimestamp = Timer.getFPGATimestamp();
        visionEstimateConsumer.accept(megatagEstimate);
    }

    public double lastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public void disableVision() {
        visionDisabled = true;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public static Pose2d flipPoseForRed(Pose2d bluePose) {
        double FIELD_LENGTH = VisionConstants.fieldLength;
        double FIELD_WIDTH = VisionConstants.fieldWidth;

        return new Pose2d(
                new Translation2d(
                        FIELD_LENGTH - bluePose.getX(),
                        FIELD_WIDTH - bluePose.getY()),
                bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }

    public void updateLogger() {
        if (this.driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/YawAngularVelocity",
                    this.driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/RollAngularVelocity",
                    this.driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/PitchAngularVelocity",
                    this.drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelX.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelX", this.accelX.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelY.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelY", this.accelY.getInternalBuffer().lastEntry().getValue());
        }
        Logger.recordOutput("RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeed());
        Logger.recordOutput("RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeed());
    }

    @Override
    protected void onTeleopStart() {
        setState(State.TRAVERSING);
        drive.setFieldPoses("Auto Path", new ArrayList<>());
        drive.setFieldPoses();

        if (!climbZeroed) {
            climbZeroed = true;
            CommandScheduler.getInstance().schedule(climb.zero());
        }
    }

    @Override
    protected void onAutonomousStart() {
        Command selected = autoChooser.get();
        if (selected != null) {
            String autoName = selected.getName();

            AutoCommands.getAutoByName(this, autoName).ifPresentOrElse(
                    (autoClass) -> registerStateCommand(State.AUTO, autoClass.getCommand(this)),
                    () -> registerStateCommand(State.AUTO, selected));
        }

        if (!climbZeroed) {
            climbZeroed = true;
            CommandScheduler.getInstance().schedule(climb.zero());
        }
        Logger.recordOutput("Auto Trajectory 3D", new Transform3d[] {});
        setState(State.AUTO);
    }

    @Override
    protected void determineSelf() {
        setState(State.TRAVERSING);
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }
        fuelSim.updateSim();
    }

    @Override
    public void update() {

        // LOGGING FOR TOF
        {
            Logger.recordOutput("Distance to Hub", TrenchZone.getDistanceToClosestShootingPose(this));
        }

        String gameState = "No Game State";
        double secondsUntilAllianceShift = 25;
        String message = DriverStation.getGameSpecificMessage();
        Optional<Alliance> teamAlliance = DriverStation.getAlliance();

        char autoWinner = (message.length() > 0) ? message.charAt(0) : ' ';
        double matchTime = DriverStation.getMatchTime();

        boolean inTransitionShift = (matchTime >= 130);
        boolean inEndGame = (matchTime <= 30);
        int currentStage = (4 - (int) ((matchTime - 30) / 25));

        if (DriverStation.isAutonomous()) {
            gameState = "Autonomous";
            secondsUntilAllianceShift = 0;

            {
                Command newAutoCommand = autoChooser.get();

                if (newAutoCommand != autoCommand && newAutoCommand != null) {
                    autoCommand = newAutoCommand;
                    String autoName = autoCommand.getName();

                    try {
                        Optional<AutoCommands.AutoClass> possibleAuto = AutoCommands.getAutoByName(this, autoName);

                        if (possibleAuto.isPresent()) {
                            List<PathPlannerPath> pathGroup = possibleAuto.get().getAutoDisplayList(this);

                            List<Pose2d> allPoses = new ArrayList<>();

                            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

                            for (PathPlannerPath path : pathGroup) {
                                for (Pose2d pose : path.getPathPoses()) {
                                    allPoses.add(isRed ? flipPoseForRed(pose) : pose);
                                }
                            }

                            Transform3d[] transformArray = allPoses.stream()
                                    .map(pose -> new Transform3d(
                                            new Translation3d(pose.getX(), pose.getY(), 0.0),
                                            new Rotation3d(0.0, 0.0, pose.getRotation().getRadians())))
                                    .toArray(Transform3d[]::new);

                            Logger.recordOutput("Auto Trajectory 3D", transformArray);
                            drive.setFieldPoses(allPoses.toArray(new Pose2d[0]));
                        } else {
                            System.out.println("OO");
                            drive.setFieldPoses();
                        }

                        {
                            PathPlannerLogging.setLogActivePathCallback((poses) -> {
                                if (poses.size() > 0) {
                                    drive.setFieldPoses();
                                }
                                drive.setFieldPoses("Auto Path", poses);

                                Transform3d[] transformArray = poses.stream()
                                        .map(pose -> new Transform3d(
                                                new Translation3d(pose.getX(), pose.getY(), 0.0),
                                                new Rotation3d(0.0, 0.0, pose.getRotation().getRadians())))
                                        .toArray(Transform3d[]::new);
                                Logger.recordOutput("Pathplanner Trajectory", transformArray);
                            });
                        }
                    } catch (Exception e) {
                        drive.setFieldPoses();
                        Elastic.sendNotification(
                                new Notification()
                                        .withTitle("Auto Mapping")
                                        .withDescription("Unable to add Auto Trajectory")
                                        .withLevel(NotificationLevel.ERROR));
                    }
                }
            }
        } else if (inTransitionShift) {
            gameState = "Transition";
            secondsUntilAllianceShift = matchTime - 130;
        } else if (inEndGame) {
            gameState = "End Game";
            secondsUntilAllianceShift = matchTime;
        } else {
            secondsUntilAllianceShift = (matchTime - 30) % 25;
            switch (currentStage) {
                case 1:
                    gameState = "Shift 1";
                    break;
                case 2:
                    gameState = "Shift 2";
                    break;
                case 3:
                    gameState = "Shift 3";
                    break;
                case 4:
                    gameState = "Shift 4";
                    break;
                default:
                    gameState = "Teleop";
                    break;
            }
        }

        if (teamAlliance.isPresent() && message.length() > 0 && !inTransitionShift && !inEndGame
                && DriverStation.isTeleop()) {
            char myColor = (teamAlliance.get() == Alliance.Red) ? 'R' : 'B';
            boolean isStageEven = (currentStage % 2 == 0);

            if (autoWinner == 'B') {
                hubActivated.set(isStageEven ? (myColor == 'B') : (myColor == 'R'));
            } else if (autoWinner == 'R') {
                hubActivated.set(isStageEven ? (myColor == 'R') : (myColor == 'B'));
            } else {
                hubActivated.set(true);
            }
        } else {
            hubActivated.set(true);
        }

        SmartDashboard.putBoolean("Game/HubActivated", hubActivated.get());
        SmartDashboard.putString("Game/GameState", gameState);
        SmartDashboard.putString("Game/ShiftCountdown", String.format("%.2f", secondsUntilAllianceShift));

        if (autoChooser.get() != null) {
            SmartDashboard.putBoolean("Robot/AutoChoosed", autoChooser.get().getName().toLowerCase().contains("game"));
        }
    }

    public enum State {
        UNDETERMINED,
        SOFT_STOP,
        TRAVERSING,
        AUTO,

        CLIMBING,
        SHOOTING,
        SHOOTING_INTAKING,
        INTAKING,
        PASSING,
        PASSING_INTAKING
    }
}