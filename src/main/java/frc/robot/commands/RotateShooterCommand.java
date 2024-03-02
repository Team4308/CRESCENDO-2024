package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.subsystems.RotateShooterSystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;


public class RotateShooterCommand extends Command {

    private final RotateShooterSystem m_subsystem;
    private final Supplier<Double> degree;

    public final PIDController pidController;

    public RotateShooterCommand(RotateShooterSystem subsystem, Supplier<Double> degree) {//degree should be in between 16-43 degrees FOR NOW
        m_subsystem = subsystem;
        this.degree = degree;

        pidController = new PIDController(Constants.PID.Shooter.kP, Constants.PID.Shooter.kI, Constants.PID.Shooter.kD);//pid not tuned

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double wantedDegree = this.degree.get();
        //0 is base postiion(16 degree)
        // 14.25925757 is max position(43 degree)
        //2200/12 gear ratio  
        //28 degrees 16-43

        double m = 16/(43-14.259);//mapping values; not percise, could use more work
        double b = (16*m);
        double outputDegree = m*wantedDegree-b;

        double encoderDegree = m_subsystem.getMotorPosition();

        double motorDegree = pidController.calculate(encoderDegree, outputDegree);
        m_subsystem.setMotorOutput(motorDegree);
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
