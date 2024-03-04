package frc.robot.commands;


import frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;


public class ClimbCommand extends Command {
  private final ClimbSubsystem m_subsystem;
  private final Supplier<Double> control;

  public ClimbCommand(ClimbSubsystem subsystem, Supplier<Double> control) {
    m_subsystem = subsystem;
    this.control = control;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.selectProfileSlot(0);
    m_subsystem.stopControllers();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double control = this.control.get();
    m_subsystem.setMotorOutput(TalonSRXControlMode.PercentOutput, control);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopControllers();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
