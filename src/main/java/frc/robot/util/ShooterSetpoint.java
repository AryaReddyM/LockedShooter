package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.SetpointLogAutoLogged;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
public class ShooterSetpoint {
    public static Optional<Double> overrideRPS = Optional.empty();

    private double shooterRPS;
    private double shooterStage1RPS = 14.4;
    private double turretRadiansFromCenter;
    private double turretFF;
    private double hoodRadians;
    private double hoodFF;
    private double height;
    private boolean isValid;

    public ShooterSetpoint(double shooterRPS, double turretRadiansFromCenter, double turretFF, double hoodRadians,
            double hoodFF, double height, boolean isValid) {
        this.shooterRPS = shooterRPS;
        this.turretRadiansFromCenter = turretRadiansFromCenter;
        this.turretFF = turretFF;
        this.hoodRadians = hoodRadians;
        this.hoodFF = hoodFF;
        this.height = height;
        this.isValid = isValid;
    }

    public ShooterSetpoint(double shooterRPS, double turretRadiansFromCenter, double turretFF, double hoodRadians,
            double hoodFF, double height) {
        this.shooterRPS = shooterRPS;
        this.turretRadiansFromCenter = turretRadiansFromCenter;
        this.turretFF = turretFF;
        this.hoodRadians = hoodRadians;
        this.hoodFF = hoodFF;
        this.height = height;
        this.isValid = true;
    }

    public static void clearOverrideRPS() {
        overrideRPS = Optional.empty();
    }

    public static void setOverrideRPS(double rps) {
        overrideRPS = Optional.of(rps);
    }

    public boolean getIsValid() {
        return this.isValid;
    }

    private static ShooterSetpoint makeSetpoint(RobotState robotState, Rotation2d robotToTargetRotation,
            Translation3d robotToTargetTranslation,
            double pitchAngleRads, double launchSpeedMetersPerSec) {
        // turret
        Rotation2d turretRotationRobotFrame = robotToTargetRotation
                .minus(robotState.getLatestFieldToRobot().getValue().getRotation());
        Rotation2d turretRotationTurretFrame = turretRotationRobotFrame
                .rotateBy(MathHelpers.kRotation2dPi).rotateBy(Rotation2d.fromDegrees(GetTuned.getNumber("Turret/ShotCorrection Degree", ShooterConstants.kTurretToShotCorrection.getDegrees())));

        // hood
        var hoodZeroedAngle = Rotation2d.fromDegrees(GetTuned.getNumber("Hood/Zeroed Angle Degrees", HoodConstants.kHoodZeroedAngleDegrees));
        double hoodAngle = hoodZeroedAngle.getRadians() - pitchAngleRads;

        // Feedfowards
        var robotSpeeds = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();

        var robotToTargetXY = new Translation2d(robotToTargetTranslation.getX(), robotToTargetTranslation.getY());

        // In this frame, x = radial component (positive towards goal)
        // y = tangential component (positive means turret needs negative lead)
        var targetFrameToRobot = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                .rotateBy(
                        robotToTargetXY.getAngle());

        var tangent = targetFrameToRobot.getY();
        var angular = robotSpeeds.omegaRadiansPerSecond;
        var distanceToTarget = robotToTargetXY.getNorm();
        var turretFF = -(angular + tangent / distanceToTarget);
        // This is the deriative of atan2 accounting for the frame that the hood is
        // defined in.
        var hoodFF = targetFrameToRobot.getX() * -robotToTargetTranslation.getZ() /
                (distanceToTarget * distanceToTarget +
                        robotToTargetTranslation.getZ() * robotToTargetTranslation.getZ());

        boolean validSetpont = true;
        double shooterRPS = launchSpeedMetersPerSec / GetTuned.getNumber("Shooter/Ball Launch Vel MPS per RPS", ShooterConstants.kBallLaunchVelMetersPerSecPerRotPerSec);
        if (shooterRPS > GetTuned.getNumber("Shooter/Stage 2 RPS Cap", ShooterConstants.kShooterStage2RPSCap)) {
            shooterRPS = GetTuned.getNumber("Shooter/Stage 2 RPS Cap", ShooterConstants.kShooterStage2RPSCap);
            validSetpont = false;
        }

        return new ShooterSetpoint(shooterRPS,
                turretRotationTurretFrame.getRadians(),
                turretFF,
                hoodAngle,
                hoodFF, robotToTargetTranslation.getZ(), validSetpont);
    }

    public static Supplier<ShooterSetpoint> passSetpointSupplier(RobotState robotState) {
        return passSetpointSupplier(() -> PassTargetFactory.generate(robotState), robotState);
    }

    public static Supplier<ShooterSetpoint> passSetpointSupplier(Supplier<Translation3d> targetPoint,
            RobotState robotState) {
        return () -> fromPassPose(targetPoint.get(), robotState);
    }

    private static ShooterSetpoint fromPassPose(Translation3d target, RobotState robotState) {
        double maxPassHeight = target.getZ();
        target = new Translation3d(target.getX(), target.getY(), 0.0);
        var fieldToRobot = robotState.getLatestFieldToRobot();
        var vRobot = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();
        Translation3d d = target.minus(
                new Translation3d(fieldToRobot.getValue().getX(), fieldToRobot.getValue().getY(),
                        GetTuned.getNumber("Shooter/Ball Release Height", ShooterConstants.kBallReleaseHeight)));
        var hoodZeroedAngle = Rotation2d.fromDegrees(GetTuned.getNumber("Hood/Zeroed Angle Degrees", HoodConstants.kHoodZeroedAngleDegrees));

        double apexHeight = maxPassHeight;
        double pitchAngleRads = 0.0;
        double launchSpeedMetersPerSec = 0.0;
        Rotation2d robotToTargetRotation = MathHelpers.kRotation2dZero;

        final int max_num_iterations = 10;
        double minApexHeight = 0.0;
        double maxApexHeight = apexHeight;
        for (int i = 0; i < max_num_iterations; ++i) {
            final double kG = -9.81;
            double vz = Math.sqrt(-2.0 * kG * (apexHeight - GetTuned.getNumber("Shooter/Ball Release Height", ShooterConstants.kBallReleaseHeight)));
            double t_apex = vz / -kG;
            double t_fall = Math.sqrt(2.0 * apexHeight / -kG);
            double t_total = t_apex + t_fall;
            
            double vx = (d.getX() - vRobot.vxMetersPerSecond * t_total) / t_total;
            double vy = (d.getY() - vRobot.vyMetersPerSecond * t_total) / t_total;

            double shotXY = Math.sqrt(vx * vx + vy * vy);
            pitchAngleRads = Math.atan2(vz, shotXY);

            double hoodAngle = hoodZeroedAngle.getRadians() - pitchAngleRads;

            if (hoodAngle < GetTuned.getNumber("Hood/Min Pos Rads", HoodConstants.kHoodMinPositionRadians)) {
                maxApexHeight = Math.min(apexHeight, maxApexHeight);
                apexHeight = (maxApexHeight - minApexHeight) / 2.0 + minApexHeight;
            } else if (apexHeight < GetTuned.getNumber("Shooter/Pass Max Apex Height", ShooterConstants.kPassMaxApexHeight)) {
                launchSpeedMetersPerSec = Math.sqrt(vz * vz + shotXY * shotXY);
                
                // Safe Rotation Assignment
                if (shotXY > 1e-6) {
                    robotToTargetRotation = new Rotation2d(vx, vy);
                } else {
                    robotToTargetRotation = MathHelpers.kRotation2dZero;
                }

                minApexHeight = Math.max(apexHeight, minApexHeight);
                apexHeight = (maxApexHeight - minApexHeight) / 2.0 + minApexHeight;
            } else {
                launchSpeedMetersPerSec = Math.sqrt(vz * vz + shotXY * shotXY);

                // Safe Rotation Assignment
                if (shotXY > 1e-6) {
                    robotToTargetRotation = new Rotation2d(vx, vy);
                } else {
                    robotToTargetRotation = MathHelpers.kRotation2dZero;
                }
                break;
            }
        }
        return makeSetpoint(robotState, robotToTargetRotation, d, pitchAngleRads, launchSpeedMetersPerSec);
    }

    public static Supplier<ShooterSetpoint> speakerSetpointSupplier(RobotState robotState) {
        return speakerSetpointSupplier(() -> BallTargetFactory.generate(robotState), robotState);
    }

    public static Supplier<ShooterSetpoint> speakerSetpointSupplier(Supplier<Translation3d> targetPoint,
            RobotState robotState) {
        return () -> fromSpeakerTarget(targetPoint.get(), robotState);
    }

    private static ShooterSetpoint fromSpeakerTarget(Translation3d target, RobotState robotState) {
        // turret
        Pose2d fieldToRobot;
        final boolean kUsePrediction = true;
        final double kPredictionLookaheadTime = 0.05;
        if (kUsePrediction) {
            fieldToRobot = robotState.getPredictedFieldToRobot(kPredictionLookaheadTime);
        } else {
            fieldToRobot = robotState.getLatestFieldToRobot().getValue();
        }
        Rotation2d robotToTargetRotation;
        Translation3d robotToTargetTranslation;
        double pitchAngleRads;

        var distanceToTarget = new Translation2d(
                target.getX() - fieldToRobot.getX(),
                target.getY() - fieldToRobot.getY()).getNorm();

        double launchSpeedRPS = 0.0;
        if (distanceToTarget < GetTuned.getNumber("Shooter/Stage 2 Max Short Range Dist", ShooterConstants.kShooterStage2MaxShortRangeDistance)) {
            launchSpeedRPS = GetTuned.getNumber("Shooter/Stage 2 RPS Short Range", ShooterConstants.kShooterStage2RPSShortRange);
        } else if (distanceToTarget > GetTuned.getNumber("Shooter/Stage 2 Min Long Range Dist", ShooterConstants.kShooterStage2MinLongRangeDistance)) {
            launchSpeedRPS = GetTuned.getNumber("Shooter/Stage 2 RPS Long Range", ShooterConstants.kShooterStage2RPSLongRange);
        } else {
            var x = (distanceToTarget - GetTuned.getNumber("Shooter/Stage 2 Max Short Range Dist", ShooterConstants.kShooterStage2MaxShortRangeDistance)) /
                    (GetTuned.getNumber("Shooter/Stage 2 Min Long Range Dist", ShooterConstants.kShooterStage2MinLongRangeDistance)
                            - GetTuned.getNumber("Shooter/Stage 2 Max Short Range Dist", ShooterConstants.kShooterStage2MaxShortRangeDistance));
            launchSpeedRPS = Util.interpolate(GetTuned.getNumber("Shooter/Stage 2 RPS Short Range", ShooterConstants.kShooterStage2RPSShortRange),
                    GetTuned.getNumber("Shooter/Stage 2 RPS Long Range", ShooterConstants.kShooterStage2RPSLongRange), x);
        }

        // if (overrideRPS.isPresent()) {
        // launchSpeedRPS = overrideRPS.get();
        // }

        double launchSpeedMetersPerSec = GetTuned.getNumber("Shooter/Ball Launch Vel MPS per RPS", ShooterConstants.kBallLaunchVelMetersPerSecPerRotPerSec) *
                launchSpeedRPS;

        boolean kUseMotionCompensation = true;
        if (kUseMotionCompensation) {
            var vRobot = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();
            var vShot = launchSpeedMetersPerSec;

            // Solve quadratic equation to obtain time of flight of ring.
            // a = vx^2+vy^2-shot_vel^2
            // b = -2*((tx-rx)*vx+(ty-ry)*vy)
            // c = (tx-rx)^2+(ty-ry)^2+dz^2
            var a = vRobot.vxMetersPerSecond * vRobot.vxMetersPerSecond +
                    vRobot.vyMetersPerSecond * vRobot.vyMetersPerSecond -
                    vShot * vShot;
            if (Math.abs(a) < Util.kEpsilon) {
                // Not a quadratic equation, cheat a little bit to make it one.
                vShot = 1.01 * vShot;
            }
            Translation3d d = target.minus(
                    new Translation3d(fieldToRobot.getX(), fieldToRobot.getY(), GetTuned.getNumber("Shooter/Ball Release Height", ShooterConstants.kBallReleaseHeight)));
            var b = -2.0 * (d.getX() * vRobot.vxMetersPerSecond +
                    d.getY() * vRobot.vyMetersPerSecond);
            var c = d.getX() * d.getX() + d.getY() * d.getY() + d.getZ() * d.getZ();

            var discriminant = b * b - 4.0 * a * c;
            if (discriminant < 0.0) {
                discriminant = 0.0;
            }
            var t = (-b - Math.sqrt(discriminant)) / (2.0 * a);
            var shot = new Translation3d((d.getX() - vRobot.vxMetersPerSecond * t) / t,
                    (d.getY() - vRobot.vyMetersPerSecond * t) / t,
                    (d.getZ() / t));
            
            robotToTargetRotation = (Math.abs(shot.getX()) < 1e-6 && Math.abs(shot.getY()) < 1e-6) ? MathHelpers.kRotation2dZero : new Rotation2d(shot.getX(), shot.getY());
            
            var xyVel = Math.sqrt(shot.getX() * shot.getX() + shot.getY() * shot.getY());
            pitchAngleRads = Math.atan2(shot.getZ(), xyVel);

            boolean kUseGravityCompensation = true;
            final double kG = -9.81;
            if (kUseGravityCompensation) {
                boolean kUseLiftCompensation = true;
                var drop = 0.5 * t * t * kG;
                if (kUseLiftCompensation) {
                    // But, v here is (distance / t), so v^2 * t^2 just becomes distance^2, which is
                    // c.
                    // That's neat, huh.
                    drop += 0.5 * GetTuned.getNumber("Shooter/Ball Lift Coeff", ShooterConstants.kBallLaunchLiftCoeff) * c;
                }
                pitchAngleRads = Math.atan2((d.getZ() - drop) / t, xyVel);
                vShot = Math.sqrt((d.getZ() - drop) * (d.getZ() - drop) / (t * t) + xyVel * xyVel);
            }
            launchSpeedMetersPerSec = vShot;
            robotToTargetTranslation = d;
        } else {
            robotToTargetRotation = (Math.abs(target.getX() - fieldToRobot.getX()) < 1e-6 && Math.abs(target.getY() - fieldToRobot.getY()) < 1e-6) ? MathHelpers.kRotation2dZero : new Rotation2d(
                    target.getX() - fieldToRobot.getX(),
                    target.getY() - fieldToRobot.getY());
            var differential_height = target.getZ() - GetTuned.getNumber("Shooter/Ball Release Height", ShooterConstants.kBallReleaseHeight);
            pitchAngleRads = Math.atan2(differential_height, distanceToTarget);
            robotToTargetTranslation = new Translation3d(
                    target.getX() - fieldToRobot.getX(),
                    target.getY() - fieldToRobot.getY(), differential_height);
        }
        return makeSetpoint(robotState, robotToTargetRotation, robotToTargetTranslation, pitchAngleRads,
                launchSpeedMetersPerSec);
    }

    public double getShooterRPS() {
        return shooterRPS;
    }

    public double getShooterStage1RPS() {
        return shooterStage1RPS;
    }

    public double getTurretRadiansFromCenter() {
        return turretRadiansFromCenter;
    }

    public double getTurretFF() {
        return turretFF;
    }

    public double getHoodRadians() {
        return hoodRadians;
    }

    public double getHoodFF() {
        return hoodFF;
    }

    public double getHeight() {
        return height;
    }

    public static SetpointLogAutoLogged getLog(ShooterSetpoint setpoint) {
        SetpointLogAutoLogged log = new SetpointLogAutoLogged();
        log.height = setpoint.getHeight();
        log.shooterRPS = setpoint.getShooterRPS();
        log.turretRadiansFromCenter = setpoint.getTurretRadiansFromCenter();
        log.turretFF = setpoint.getTurretFF();
        log.hoodRadians = setpoint.getHoodRadians();
        log.hoodFF = setpoint.getHoodFF();
        log.isValid = setpoint.isValid;
        
        return log;
    }
}