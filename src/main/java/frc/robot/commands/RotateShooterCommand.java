package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.subsystems.RotateShooterSystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import ca.team4308.absolutelib.math.DoubleUtils;

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
        //0 is base position(16 degree)
        //14.25925757 is max revolutions(43 degree)
        //2200/12 gear ratio  

        double m = (Constants.Shooter.motorStartRevolutions-Constants.Shooter.motorEndRevolutions)/(Constants.Shooter.shooterStartDegree-Constants.Shooter.shooterEndDegree);
        double b = Constants.Shooter.motorStartRevolutions-(Constants.Shooter.shooterStartDegree*m);
        double outputDegree = m*wantedDegree+b;

        double encoderDegree = m_subsystem.getMotorPosition();

        double motorDegree = DoubleUtils.clamp(pidController.calculate(encoderDegree, outputDegree), -1.0, 1.0);
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
