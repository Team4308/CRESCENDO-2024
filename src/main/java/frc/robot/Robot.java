// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;

import swervelib.parser.SwerveParser;

public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    m_robotContainer = new RobotContainer();
    m_robotContainer.ewyellowerrors();

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop immediately when disabled, but then also let it be pushed more
    for (LogSubsystem subsystem : m_robotContainer.subsystems) {
      Shuffleboard.getTab("Log").add(subsystem.log());
    }
    disabledTimer = new Timer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.stopRumble();
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledActions();
    if (disabledTimer.hasElapsed(Constants.Swerve.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.zeroGyroOnTeleop();
    m_robotContainer.setMotorBrake(true);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    try {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
