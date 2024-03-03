package frc.robot.commands;

import frc.robot.subsystems.RotateShooterSystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateShooterCommand extends Command {

    private final RotateShooterSystem m_subsystem;
    private final Double degree;


    public RotateShooterCommand(RotateShooterSystem subsystem, Double degree) {
        m_subsystem = subsystem;
        this.degree = degree;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_subsystem.setMotorPosition(degree);
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
