package frc.robot.commands.Pivot;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class ToAngle extends Command {
  private Timer m_timer = new Timer();
  private final Supplier<Double> control;
  protected final PivotSubsystem m_subsystem;
  private State initialState;

  public ToAngle(PivotSubsystem subsystem, Supplier<Double> targetAngle) {
    control = targetAngle;
    m_subsystem = subsystem;

    this.addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    initialState = m_subsystem.getCurrentState();
  }

  @Override
  public void execute() {
    var nextState = m_subsystem.profiler.calculate(m_timer.get(), initialState, 
                    new TrapezoidProfile.State(control.get(), 0));
    m_subsystem.changeState(nextState);
    m_subsystem.setMotorOutput();
  }

  @Override
  public boolean isFinished() {
    // return m_profiler.isFinished(m_timer.get()) && m_arm.atSetpoint();
    return m_subsystem.profiler.isFinished(m_timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

}