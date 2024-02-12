// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexSystem;

public class RobotContainer {
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  //Subsystems
  private final IndexSystem m_indexSystem;

  //Controllers
  public final XBoxWrapper stick1 = new XBoxWrapper(0);

  public RobotContainer() {
    //Subsystem Instantiations
    m_indexSystem = new IndexSystem();

    configureBindings();
  }

  private void configureBindings() {
    stick1.RB.whileTrue(new InstantCommand(() -> m_indexSystem.setIndexOutput(1.0)));
  }
}