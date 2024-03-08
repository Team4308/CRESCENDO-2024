package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSystem;

public class LEDCommand extends Command {
    private final LEDSystem m_subsystem;
    private final Supplier<Double> control;

    // Init
    public LEDCommand(LEDSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double control = this.control.get();

        if (control == 0.0){
            m_subsystem.colourOutputShooter(control);
        } else {
            m_subsystem.setOutput(control);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

}