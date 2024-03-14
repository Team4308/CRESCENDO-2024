package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class BeambreakCommand extends Command {
    private final Supplier<Boolean> control;

    public BeambreakCommand(Supplier<Boolean> control) {
        this.control = control;
    }
    
    @Override
    public boolean isFinished() {
        Boolean control = this.control.get();
        if (control) {
            return false;
        }
        return true;
    }
}
