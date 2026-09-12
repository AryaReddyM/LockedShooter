package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Drives the robot through the centre pile with the intake running, which is what a driver
 * actually does. Collecting used to stop dead about a second in: the fuel simulation swallows
 * every ball inside the intake's box on every physics subtick, so the robot filled to capacity
 * almost instantly and then silently pushed everything around instead.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DriveIntoPileTest {
  private RobotState robotState;

  @BeforeAll
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    robotState = new RobotContainer().getRobotState();
    frc.robot.util.state.SubsystemManagerFactory.getInstance().registerSubsystem(robotState);
    frc.robot.util.state.SubsystemManagerFactory.getInstance().notifyTeleopStart();
  }

  @Test
  void collectingKeepsWorkingForAsLongAsTheDriverHoldsIt() {
    robotState.getDrive().setPose(new Pose2d(6.6, 4.02, Rotation2d.kZero));
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.intake(robotState));
    for (int i = 0; i < 30; i++) {
      CommandScheduler.getInstance().run();
    }
    assertEquals(Intake.State.INTAKE, robotState.getIntake().getState());

    int start = robotState.getSimFuelCount();
    int onFieldStart = robotState.getFuelSim().getFuelCount();

    int firstHalf = drive(75);
    int midway = robotState.getSimFuelCount();
    int secondHalf = drive(75);
    int end = robotState.getSimFuelCount();

    assertTrue(firstHalf > 0, "nothing was collected driving into the pile");
    assertTrue(
        secondHalf > 0,
        "collecting stopped part way through: picked up "
            + firstHalf
            + " then "
            + secondHalf
            + ", ending at "
            + end
            + " held (midway "
            + midway
            + ")");

    // Every ball the robot holds came off the field, none were conjured up.
    assertEquals(
        end - start,
        onFieldStart - robotState.getFuelSim().getFuelCount(),
        "held count and field count disagree");

    // A real intake feeds one ball at a time. Three seconds of driving should not inhale
    // the entire pile.
    assertTrue(
        end - start < 40,
        "collected " + (end - start) + " balls in 3 s, the intake is vacuuming the field");
  }

  @Test
  void resetFuelPutsTheFieldBack() {
    int fullField = robotState.getFuelSim().getFuelCount();

    // Drive through the pile and score some, so the field is genuinely disturbed.
    robotState.getDrive().setPose(new Pose2d(6.6, 4.02, Rotation2d.kZero));
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(frc.robot.commands.ActionCommands.intake(robotState));
    for (int i = 0; i < 30; i++) {
      CommandScheduler.getInstance().run();
    }
    drive(100);
    assertTrue(
        robotState.getFuelSim().getFuelCount() < fullField, "the drive should have removed fuel");

    CommandScheduler.getInstance().cancelAll();
    robotState.resetFuel();

    assertEquals(
        fullField,
        robotState.getFuelSim().getFuelCount(),
        "reset should put every piece of fuel back on the field");
    assertEquals(5, robotState.getSimFuelCount(), "reset should leave the robot with its preload");
    assertEquals(0, frc.robot.util.sim.FuelSim.Hub.BLUE_HUB.getScore(), "blue score should zero");
    assertEquals(0, frc.robot.util.sim.FuelSim.Hub.RED_HUB.getScore(), "red score should zero");
  }

  /** Runs the given number of loops driving forwards, and returns how many balls were gained. */
  private int drive(int loops) {
    int before = robotState.getSimFuelCount();
    for (int i = 0; i < loops; i++) {
      robotState.getDrive().runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
      CommandScheduler.getInstance().run();
    }
    return robotState.getSimFuelCount() - before;
  }
}
