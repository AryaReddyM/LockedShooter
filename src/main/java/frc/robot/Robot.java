// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.state.SubsystemManagerFactory;

public class Robot extends LoggedRobot {  
  private final RobotState robotState;

  public Robot() {

    Logger.recordMetadata("ROBOT", "2026 Recharge");
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    
    StatusLogger.disableAutoLogging();

    switch (Constants.currentMode) {
    case REAL:
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      break;
    case SIM:
      Logger.addDataReceiver(new NT4Publisher());
      break;
    case REPLAY:
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      break;
    }
    
    Logger.start();

    robotState = new RobotState();
    SubsystemManagerFactory.getInstance().registerSubsystem(robotState);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotState.updateLogger();
  }

  @Override
  public void disabledInit() {
    SubsystemManagerFactory.getInstance().disableAllSubsystems();
  }

  @Override
  public void simulationPeriodic() {
    robotState.updateSimulation();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SubsystemManagerFactory.getInstance().notifyAutonomousStart();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    SubsystemManagerFactory.getInstance().notifyTeleopStart();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    SubsystemManagerFactory.getInstance().notifyTestStart();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
