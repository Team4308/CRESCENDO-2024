package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.RotateShooterSystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootInAmpCommand extends Command {

    private final RotateShooterSystem m_subsystem;

    public ShootInAmpCommand(RotateShooterSystem subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_subsystem.setMotorPosition(Constants.gamePieces.amp.angleToshoot);
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