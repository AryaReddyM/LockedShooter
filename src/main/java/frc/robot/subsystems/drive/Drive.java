// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import dev.doglog.DogLog;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.logging.Elastic;
import frc.robot.util.logging.Elastic.Notification;
import frc.robot.util.state.StateMachine;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachine<Drive.State> implements DriveIO {
  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();

  public final Field2d fieldPose = new Field2d();

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
      lastModulePositions, Pose2d.kZero);

  /**
   * Wheels-only pose, with no vision correction folded in.
   *
   * <p>In simulation there is no wheel slip, so this is the robot's true position. The vision
   * simulation renders its AprilTags from this rather than from the estimator, because feeding the
   * estimator's own output back in as ground truth is a closed loop: any small bias in the solve
   * gets re-observed, re-applied, and the pose walks away on its own. It drifted about 19 cm in
   * five seconds sitting disabled before this was split out.
   */
  private final SwerveDriveOdometry wheelOdometry = new SwerveDriveOdometry(kinematics, rawGyroRotation,
      lastModulePositions, Pose2d.kZero);

  public static double kABDriveP = DriveConstants.kABDriveP;
  public static double kABDriveI = DriveConstants.kABDriveI;
  public static double kABDriveD = DriveConstants.kABDriveD;

  public static double kABTurnP = DriveConstants.kABTurnP;
  public static double kAPTurnI = DriveConstants.kABTurnI;
  public static double kAPTurnD = DriveConstants.kABTurnD;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    super("Drive", State.UNDETERMINED, State.class);
    this.gyroIO = gyroIO;

    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    if (RobotBase.isReal()) {
      SparkOdometryThread.getInstance().start();
    }

    configureAutobuilder();

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    registerStateTransitions();
    registerStateCommands();
    enable();

    SmartDashboard.putData("Swerve Zero", Commands.runOnce(
        () -> this.setPose(
            new Pose2d(this.getPose().getTranslation(), Rotation2d.kZero)),
        this)
        .ignoringDisable(true));
        
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
      }
    });
  }

  private void registerStateTransitions() {
    addOmniTransitions(State.IDLE, State.CROSSED, State.ALIGNING, State.PATHFINDING, State.SLOW, State.TRAVERSING,
        State.TRAVERSING_AT_ANGLE, State.UNDETERMINED);
  }

  private void registerStateCommands() {
    registerStateCommand(State.IDLE, new InstantCommand(() -> stop()));
    registerStateCommand(State.CROSSED, new InstantCommand(() -> stopWithX()));
  }

  private void configureAutobuilder() {
    Elastic.sendNotification(new Notification().withTitle("Auto Builder").withDescription("Auto builder reset"));
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(kABDriveP, kABDriveI, kABDriveD), new PIDConstants(kABTurnP, kABTurnI, kABTurnD)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  @Override
  public void update() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);

    {
      driveInputs.modStates = getModuleStates();
      driveInputs.currentPose = getPose();
      driveInputs.currentPose3d = new Pose3d(driveInputs.currentPose);

      fieldPose.setRobotPose(driveInputs.currentPose);
    }

    Logger.processInputs("Drive/Gyro", gyroInputs);
    Logger.processInputs("Drive/DriveBase", driveInputs);
    SmartDashboard.putData(fieldPose);

    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      wheelOdometry.update(rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);

    driveInputs.chassieSpeeds = discreteSpeeds;
    driveInputs.optimizedModStates = setpointStates;
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Pose from the wheels alone, with no vision applied. In simulation this is ground truth; on the
   * real robot it is the raw odometry, useful for seeing how far vision is moving things.
   */
  @AutoLogOutput(key = "Odometry/WheelsOnly")
  public Pose2d getWheelOdometryPose() {
    return wheelOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    wheelOdometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
    Elastic.sendNotification(
        new Notification().withTitle("Pose Reset").withDescription("Pose has been set to a new custom one"));
  }

  public void zeroGyro() {
    gyroIO.getPiegon().setYaw(0);
  }

  public void setTargetPose(Pose2d pose) {
    driveInputs.goalPose = pose;
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    if (getState() == State.SLOW) {
      return slowSpeedMetersPerSec;
    }
    return maxSpeedMetersPerSec;
  }

  public double getMaxAngularSpeedRadPerSec() {
    if (getState() == State.SLOW) {
      return slowSpeedMetersPerSec / driveBaseRadius;
    }
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  public GyroIOInputsAutoLogged getGyroIOInputs() {
    return gyroInputs;
  }

  public DriveIOInputsAutoLogged getDriveIOInputs() {
    return driveInputs;
  }

  public void determineSelf() {
    setState(State.TRAVERSING);
  }

  @Override
  public void onTeleopStart() {
    setFieldPoses();
  }

  public void setFieldPoses(Pose2d... poses) {
    fieldPose.getObject("mainTrajectory").setPoses(poses);
  }

  public void setFieldPoses(String object, List<Pose2d> poses) {
    fieldPose.getObject(object).setPoses(poses);
  }

  public enum State {
    UNDETERMINED,

    IDLE,
    CROSSED,
    TRAVERSING,
    TRAVERSING_AT_ANGLE,

    PATHFINDING,
    ALIGNING,

    // Flags,
    SLOW,
  }
}