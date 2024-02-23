package frc.robot;

import java.util.ArrayList;

import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;

import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.wrapper.LogSubsystem;

public class RobotContainer {
  //Subsystems
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();
  private final ShooterSubsystem m_shooterSubsystem;

  //Commands
  private final ShooterCommand ShooterCommand;

  //Controllers
  public final XBoxWrapper stick1 = new XBoxWrapper(Constants.Mapping.Controllers.kStick1);
  public final XBoxWrapper stick2 = new XBoxWrapper(Constants.Mapping.Controllers.kStick2);

  public RobotContainer() {
    m_shooterSubsystem = new ShooterSubsystem();

    subsystems.add(m_shooterSubsystem);

    ShooterCommand = new ShooterCommand(m_shooterSubsystem, () -> shooterControl());
        
    m_shooterSubsystem.setDefaultCommand(ShooterCommand);

    configureBindings();
  }

  private void configureBindings() {
  }

  public double shooterControl() {
    return stick2.getRightTrigger();
  }
}
