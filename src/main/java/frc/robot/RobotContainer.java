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
import frc.robot.Commands.DriveCommand;
import frc.robot.Subsystems.DriveSystem;

public class RobotContainer {
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  //Subsystems
  private final DriveSystem m_driveSystem;

  //Commands
  private final DriveCommand driveCommand;
  
  //Controllers
  public final XBoxWrapper stick1 = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  //Auto
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  public RobotContainer() {
    //Subsystem Instantiations
    m_driveSystem = new DriveSystem();
    subsystems.add(m_driveSystem);

    //Command Instantiations
    driveCommand = new DriveCommand(m_driveSystem, () -> getDriveControl());
    m_driveSystem.setDefaultCommand(driveCommand);

    SmartDashboard.putData(autoCommandChooser);

    configureBindings();
  }

  private void configureBindings() {
    stick1.B.onTrue(new InstantCommand(() -> m_driveSystem.resetAngle(), m_driveSystem));
  }

  public Vector2 getDriveControl() {
    double throttle = DoubleUtils.normalize(stick1.getLeftY());

    double turn = DoubleUtils.normalize(stick1.getRightX());

    throttle *= 0.7;
    turn *= 0.7;

    if(stick1.RB.getAsBoolean()){
      throttle /= 1.5;
      turn /= 2;
    }

    Vector2 control = new Vector2(turn, throttle);
    control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    control = JoystickHelper.scaleStick(control, Constants.Config.Input.Stick.kInputScale);
    control = JoystickHelper.clampStick(control);

    return control;
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }
}