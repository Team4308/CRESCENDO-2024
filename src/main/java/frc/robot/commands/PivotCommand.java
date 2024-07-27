package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotCommand extends Command {

    private final PivotSubsystem m_subsystem;
    private final Supplier<Double> control;

    public PivotCommand(PivotSubsystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double control = this.control.get();
        m_subsystem.setMotorPosition(control);
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
