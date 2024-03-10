package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class ShooterCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final Supplier<Double> control;

  public ShooterCommand(ShooterSubsystem subsystem, Supplier<Double> control) {
    m_subsystem = subsystem;
    this.control = control;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.selectProfileSlot(0);
    m_subsystem.stopControllers();
  }

  @Override
  public void execute() {
    double control = this.control.get();//converting to rpm
    m_subsystem.setMotorOutput(control);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopControllers();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
