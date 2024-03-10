package frc.robot.subsystems;

import java.util.ArrayList;

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

        motor.configFactoryDefault(Constants.Generic.timeoutMs);
        motor.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
        motor.configClosedloopRamp(0.1, Constants.Generic.timeoutMs);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
        motor.changeMotionControlFramePeriod(5);
        motor.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
        motor.enableVoltageCompensation(true);

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