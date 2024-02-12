package frc.robot.subsystems;

import java.util.*;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;


import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;


public class ShooterSubsystem extends LogSubsystem {
    public final TalonFX motor1;
    public final TalonFX motor2;

    public ShooterSubsystem() {
        // Create Motor Controller Objects
        motor1 = new TalonFX(Constants.Mapping.ShooterMotor.kMotor1);
        motor2 = new TalonFX(Constants.Mapping.ShooterMotor.kMotor2);

        motor1.setInverted(TalonFXInvertType.Clockwise);
        motor2.follow(motor1);
        motor2.setInverted(TalonFXInvertType.FollowMaster);

        Collection<TalonFX> motors = new ArrayList<TalonFX>(); 
        motors.add(motor1);
        motors.add(motor2);
        for (TalonFX motor : motors) {
            motor.configFactoryDefault(Constants.Generic.kTimeoutMs);
            motor.configNeutralDeadband(0.05, Constants.Generic.kTimeoutMs);
            motor.changeMotionControlFramePeriod(5);
            motor.setNeutralMode(NeutralMode.Coast);
            motor.configVoltageCompSaturation(12.5, Constants.Generic.kTimeoutMs);
            motor.enableVoltageCompensation(true);
        }

        // Reset
        stopControllers();
        resetSensors();
    }

    /**
     * Misc Stuff
     */
    public void setMotorOutput(TalonFXControlMode mode, double percent) {
        motor1.set(mode, percent);
    }

    public void selectProfileSlot(int slot) {
        motor1.selectProfileSlot(slot, 0);
    }

    public void stopControllers() {
        motor1.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    // Sensor Reset
    public void resetSensors() {
        motor1.setSelectedSensorPosition(0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}
