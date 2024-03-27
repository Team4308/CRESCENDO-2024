package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class IntakeSystem extends MotoredSubsystem {
    public final TalonSRX motor;
    public boolean state;

    public IntakeSystem() {
        // Setup and Add Controllers

        //change later
        motor = new TalonSRX(Constants.Mapping.Intake.intakeMotor);

        motor.setNeutralMode(NeutralMode.Brake);

        stopControllers();
    }

    public void setIntakeOutput(TalonSRXControlMode mode, double val) {
        motor.set(mode, val);
    }

    public void stopControllers() {
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}