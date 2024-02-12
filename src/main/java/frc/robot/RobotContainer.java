// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSystem;

public class RobotContainer {
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  //Subsystems
  private final IntakeSystem m_intakeSystem;

  //Commands
  private final IntakeCommand intakeCommand;
  

  //Controllers
  public final XBoxWrapper stick1 = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  //Auto
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  public RobotContainer() {
    //Subsystem Instantiations
    m_intakeSystem = new IntakeSystem();
    subsystems.add(m_intakeSystem);

    //Command Instantiations
    intakeCommand = new IntakeCommand(m_intakeSystem, () -> getIntakeControl());
    m_intakeSystem.setDefaultCommand(intakeCommand);

    SmartDashboard.putData(autoCommandChooser);
    configureBindings();
  }

  private void configureBindings() {
    stick2.A.whileTrue(new IntakeCommand(m_intakeSystem, () -> getIntakeControl()));
  }

  public double getIntakeControl() {
    return 0.5; // change to - or + depending on ccw/cw on the robot
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }
}