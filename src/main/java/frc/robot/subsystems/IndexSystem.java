package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IndexSystem extends MotoredSubsystem {
    public final TalonSRX motor;
    public final DigitalInput shooterBeambreak;
    public boolean state;

    public IndexSystem() {
        // Setup Controllers
        motor = new TalonSRX(Constants.Mapping.Index.indexMotor);
        motor.setNeutralMode(NeutralMode.Brake);

        shooterBeambreak = new DigitalInput(Constants.Mapping.Shooter.beambreak);

        stopControllers();
    }

    public void setIndexOutput(double output) {
        motor.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void stopControllers() {
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    public boolean getBeambreak() {
        return shooterBeambreak.get();
    }

    @Override
    public Sendable log() {
        return this;
    }
}