package frc.robot.commands.Pivot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;


public class ManualPivot extends Command {
  private final PivotSubsystem m_subsystem;
  private final Supplier<Double> m_angleSupplier;

  public ManualPivot(PivotSubsystem subsystem, Supplier<Double> angleSupplier) {
    m_angleSupplier = angleSupplier;
    m_subsystem = subsystem;

    this.addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.stopControllers();
  }

  @Override
  public void execute() {
    double newShooterSetpoint = m_subsystem.getCurrentPosition() + this.m_angleSupplier.get();
    m_subsystem.changeSetpoint(newShooterSetpoint);
    m_subsystem.setMotorOutput();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopControllers();
  }
}
