package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class IndexSystem extends MotoredSubsystem {
    public final TalonSRX motor;
    public boolean state;

    public IndexSystem() {
        // Setup Controllers
        motor = new TalonSRX(Constants.Mapping.Index.indexMotor);

        stopControllers();
    }

    public void setIndexOutput(double output) {
        motor.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void stopControllers() {
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}