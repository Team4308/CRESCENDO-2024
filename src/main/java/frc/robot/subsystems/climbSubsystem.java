package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class climbSubsystem extends LogSubsystem {

    // Motor Controllers
    public final TalonSRX motor1;
    private final TalonSRX motor2;

    // Init
    public climbSubsystem() {

        // Create Motor Controller Objects
        motor1 = new TalonSRX(Constants.Mapping.climbMotors.motor1);
        motor2 = new TalonSRX(Constants.Mapping.climbMotors.motor2);

        
        // Configure Motor Controllers to Factory Settings
        motor1.configFactoryDefault(Constants.Generic.timeoutMs);
        motor2.configFactoryDefault(Constants.Generic.timeoutMs);

        // Configure Motor Controllers
        // Note: These are unneeded for this case in specific,
        // but in most cases, they will be used
        motor1.setNeutralMode(NeutralMode.Coast);
        motor1.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
        motor1.changeMotionControlFramePeriod(5);
        motor1.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
        motor1.enableVoltageCompensation(true);
        motor2.setNeutralMode(NeutralMode.Coast);
        motor2.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
        motor2.changeMotionControlFramePeriod(5);
        motor2.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
        motor2.enableVoltageCompensation(true);
        // Reset
        stopControllers();
        resetSensors();
    }

    /**
     * Misc Stuff
     */
    public void setMotorOutput(TalonSRXControlMode mode, double percent) {
        motor1.set(mode, percent);
        motor2.set(mode, percent);
    }

    public void selectProfileSlot(int slot) {
        motor1.selectProfileSlot(slot, 0);
        motor2.selectProfileSlot(slot, 0);
    }

    public void stopControllers() {
        motor1.set(TalonSRXControlMode.PercentOutput, 0.0);
        motor2.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    // Sensor Reset
    public void resetSensors() {
        motor1.setSelectedSensorPosition(0);
        motor2.setSelectedSensorPosition(0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}