package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.subsystems.RotateShooterSystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateShooterCommand extends Command {

    private final RotateShooterSystem m_subsystem;
    private final Supplier<Double> control;

    public RotateShooterCommand(RotateShooterSystem subsystem, Supplier<Double> control) {
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
