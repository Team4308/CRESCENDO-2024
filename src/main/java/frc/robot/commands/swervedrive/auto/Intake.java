package frc.robot.commands.swervedrive.auto;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

public class Intake extends Command {
    private final IntakeSystem m_subsystem;
    private final double control;

    int withinThresholdLoops;

    public Intake(double control, IntakeSystem subsystem) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(this.m_subsystem);

        withinThresholdLoops = 0;
    }

    @Override
    public void initialize() {
        m_subsystem.stopControllers();
    }

    @Override
    public void execute() {
        m_subsystem.setIntakeOutput(TalonSRXControlMode.PercentOutput, 0.5);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 5);
    }
}
