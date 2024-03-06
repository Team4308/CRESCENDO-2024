package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

import java.util.ArrayList;

public class IndexSystem extends MotoredSubsystem {
    public final TalonSRX motor1;
    public final TalonSRX motor2;
    private ArrayList<TalonSRX> controllersSRX = new ArrayList<TalonSRX>();
    public boolean state;

    public IndexSystem() {
        // Setup Controllers
        motor1 = new TalonSRX(Constants.Mapping.Index.indexMotor);
        motor2 = new TalonSRX(Constants.Mapping.Index.indexMotor);

        controllersSRX.add(motor1);
        controllersSRX.add(motor2);


        for (TalonSRX talon : controllersSRX) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
            talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
            talon.configClosedloopRamp(0.1, Constants.Generic.timeoutMs);
            talon.setNeutralMode(NeutralMode.Brake);
            talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
            talon.changeMotionControlFramePeriod(5);
            talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
            talon.enableVoltageCompensation(true);
        }

        stopControllers();
    }

    public void setIndexOutput(double output) {
        motor1.set(TalonSRXControlMode.PercentOutput, output);
        motor2.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void stopControllers() {
        motor1.set(TalonSRXControlMode.PercentOutput, 0.0);
        motor2.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}