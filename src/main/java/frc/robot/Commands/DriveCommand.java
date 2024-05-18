package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveSystem;

public class DriveCommand extends Command {
    private final DriveSystem m_subsystem;
    private final Supplier<Vector2> control;

    // Init
    public DriveCommand(DriveSystem subsystem, Supplier<Vector2> control) {
        m_subsystem = subsystem;
        this.control = control;

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot);
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Vector2 control = this.control.get();

        double leftTargetRPM = -control.y * Constants.DynConfig.Drive.VelocityDriveRPM;
        double rightTargetRPM = -control.y * Constants.DynConfig.Drive.VelocityDriveRPM;

        leftTargetRPM += control.x * Constants.DynConfig.Drive.VelocityDriveRPM;
        rightTargetRPM += -control.x * Constants.DynConfig.Drive.VelocityDriveRPM;

        leftTargetRPM = DoubleUtils.clamp(leftTargetRPM, -Constants.DynConfig.Drive.VelocityDriveRPM,
                Constants.DynConfig.Drive.VelocityDriveRPM);
        rightTargetRPM = DoubleUtils.clamp(rightTargetRPM, -Constants.DynConfig.Drive.VelocityDriveRPM,
                Constants.DynConfig.Drive.VelocityDriveRPM);

        double leftTargetUnitsPS = (leftTargetRPM / 600.0)
                * (Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation);
        double rightTargetUnitsPS = (rightTargetRPM / 600.0)
                * (Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation);

        m_subsystem.setMotorOutput(TalonSRXControlMode.Velocity.toControlMode(), leftTargetUnitsPS,
                rightTargetUnitsPS);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}