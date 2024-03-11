package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.RotateShooterSystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootInAmpCommand extends Command {

    private final RotateShooterSystem m_rotateSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public ShootInAmpCommand(RotateShooterSystem rotateSubsystem, ShooterSubsystem shooterSubsystem) {
        m_rotateSubsystem = rotateSubsystem;
        m_shooterSubsystem = shooterSubsystem;

        addRequirements(rotateSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_rotateSubsystem.setMotorPosition(Constants.GamePieces.amp.angleToshoot);
        m_shooterSubsystem.setMaxSpeed(Constants.GamePieces.amp.speedToShoot);//idk
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setMaxSpeed(10000);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}