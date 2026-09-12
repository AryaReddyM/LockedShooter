package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import java.util.function.BiConsumer;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Drives every simulated mechanism to a setpoint and checks it actually gets there, in the units
 * the subsystem talks to it in.
 */
class MechanismSimTest {
  private static final int kSettleLoops = 250; // 5 seconds at 20 ms

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    assertEquals(
        Constants.Mode.SIM, Constants.currentMode, "tests must run with the sim IO implementations");
  }

  private static double settle(MotorIO io, BiConsumer<MotorIO, Double> command, double setpoint) {
    MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
    for (int i = 0; i < kSettleLoops; i++) {
      command.accept(io, setpoint);
      io.updateInputs(inputs);
    }
    return inputs.positionRad;
  }

  private static double settleVelocity(MotorIO io, double setpoint) {
    MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
    for (int i = 0; i < kSettleLoops; i++) {
      io.setMotionMagicVelocity(setpoint);
      io.updateInputs(inputs);
    }
    return inputs.velocityRadPerSec;
  }

  @Test
  void turretReachesCommandedAngleInRadians() {
    MotorIO io = TurretConstants.createIO();
    double target = 1.25;
    double actual = settle(io, (m, sp) -> m.setMotionMagicPosition(sp), target);
    assertEquals(target, actual, 0.02, "turret should track a radian setpoint");
  }

  @Test
  void turretReachesItsBackwardSoftLimit() {
    MotorIO io = TurretConstants.createIO();
    double target = TurretConstants.kBackwardSoftLimit;
    double actual = settle(io, (m, sp) -> m.setMotionMagicPosition(sp), target);
    assertEquals(target, actual, 0.02, "turret should reach the far end of its travel");
  }

  @Test
  void hoodReachesCommandedAngleInRadians() {
    MotorIO io = HoodConstants.createIO();
    double target = HoodConstants.kHoodMaxLimit;
    double actual =
        settle(
            io,
            (m, sp) -> m.setMotionMagicPosition(sp, HoodConstants.kHoodG),
            target);
    assertEquals(target, actual, 0.01, "hood should track a radian setpoint");
  }

  @Test
  void flywheelReachesCommandedSurfaceSpeed() {
    MotorIO io = FlywheelConstants.createIO();
    double target = 20.0; // m/s of surface speed
    double actual = settleVelocity(io, target);
    assertEquals(target, actual, FlywheelConstants.kFlywheelSpeedTolerance, "flywheel surface speed");
  }

  @Test
  void flywheelCanReachTheSpeedNeededForALongShot() {
    MotorIO io = FlywheelConstants.createIO();
    double target = FlywheelConstants.exitVelocityToSurfaceSpeed(14.0);
    double actual = settleVelocity(io, target);
    assertEquals(target, actual, FlywheelConstants.kFlywheelSpeedTolerance, "long shot surface speed");
  }

  @Test
  void intakeExtensionReachesSetpointsInDegrees() {
    MotorIO io = IntakeConstants.createExtensionIO();
    double stowed = settle(io, (m, sp) -> m.setMotionMagicPosition(sp), IntakeConstants.kExtensionStowSetpoint);
    assertEquals(IntakeConstants.kExtensionStowSetpoint, stowed, 1.0, "stow position, in degrees");

    double deployed =
        settle(io, (m, sp) -> m.setMotionMagicPosition(sp), IntakeConstants.kExtensionIntakeSetpoint);
    assertEquals(IntakeConstants.kExtensionIntakeSetpoint, deployed, 1.0, "intake position, in degrees");
  }

  @Test
  void intakeRollersReachCommandedSpeed() {
    MotorIO io = IntakeConstants.createRollersIO();
    double actual = settleVelocity(io, IntakeConstants.kRollerIntakeSpeed);
    assertEquals(IntakeConstants.kRollerIntakeSpeed, actual, 2.0, "roller speed");
  }

  @Test
  void hopperReachesCommandedSpeed() {
    MotorIO io = HopperConstants.createIO();
    double actual = settleVelocity(io, HopperConstants.kHopperShootSpeed);
    assertEquals(HopperConstants.kHopperShootSpeed, actual, 1.0, "hopper speed");
  }

  @Test
  void kickerReachesCommandedSpeed() {
    MotorIO io = KickerConstants.createIO();
    double actual = settleVelocity(io, KickerConstants.kKickerShootSpeed);
    assertEquals(
        KickerConstants.kKickerShootSpeed, actual, KickerConstants.kKickerSpeedTolerance, "kicker speed");
  }

  @Test
  void climbReachesCommandedPositionInRotations() {
    MotorIO io = ClimbConstants.createIO();
    double actual = settle(io, (m, sp) -> m.setMotionMagicPosition(sp), ClimbConstants.kClimbUpPos);
    assertEquals(ClimbConstants.kClimbUpPos, actual, 0.05, "climb position, in output rotations");
  }

  @Test
  void elevatorHoldsHeightAgainstGravity() {
    MotorIO io = ElevatorConstants.createIO();
    double actual =
        settle(
            io,
            (m, sp) -> m.setMotionMagicPosition(sp, ElevatorConstants.kElevatorG),
            ElevatorConstants.kHighHeight);
    assertEquals(ElevatorConstants.kHighHeight, actual, ElevatorConstants.kEpsilon, "elevator height");
  }

  @Test
  void stoppingLeavesTheMechanismUncommanded() {
    MotorIO io = FlywheelConstants.createIO();
    settleVelocity(io, 20.0);

    MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
    for (int i = 0; i < kSettleLoops; i++) {
      io.stop();
      io.updateInputs(inputs);
    }
    assertEquals(0.0, inputs.appliedVolts, 1e-9, "stop should mean zero volts");
    // Nothing brakes the wheel but the motor's own back EMF, so it coasts rather than
    // stopping dead; five seconds should still take most of the speed out of it.
    assertTrue(
        Math.abs(inputs.velocityRadPerSec) < 0.1 * 20.0,
        "flywheel should coast down, was " + inputs.velocityRadPerSec);
  }
}
