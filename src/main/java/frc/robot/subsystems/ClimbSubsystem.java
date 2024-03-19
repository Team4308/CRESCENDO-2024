package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class ClimbSubsystem extends LogSubsystem {

    // Motor Controllers
    public final TalonSRX motor1;
    private final TalonSRX motor2;

    // Init
    public ClimbSubsystem() {

        // Create Motor Controller Objects
        motor1 = new TalonSRX(Constants.Mapping.ClimbMotors.motor1);
        motor2 = new TalonSRX(Constants.Mapping.ClimbMotors.motor2);

        
        // Configure Motor Controllers to Factory Settings
        motor1.configFactoryDefault(Constants.Generic.timeoutMs);
        motor2.configFactoryDefault(Constants.Generic.timeoutMs);

        // Configure Motor Controllers
        // Note: These are unneeded for this case in specific,
        // but in most cases, they will be used
        motor1.setNeutralMode(NeutralMode.Brake);
        motor2.setNeutralMode(NeutralMode.Brake);
        motor1.setInverted(false);
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
