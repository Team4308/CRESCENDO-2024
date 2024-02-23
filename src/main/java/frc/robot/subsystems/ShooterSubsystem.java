package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;


public class ShooterSubsystem extends LogSubsystem {
    public final TalonFX rightMaster;
    public final TalonFX leftSlave;

    private final DutyCycleOut motorOut;

    private final TalonFXConfiguration rightMasterConfiguration;
    private final TalonFXConfiguration leftSlaveConfiguration;

    public ShooterSubsystem() {
        // Create Motor Controller Objects
        rightMaster = new TalonFX(Constants.Mapping.ShooterMotor.kMotor1);
        leftSlave = new TalonFX(Constants.Mapping.ShooterMotor.kMotor2);

        motorOut = new DutyCycleOut(0);

        rightMasterConfiguration = new TalonFXConfiguration();
        leftSlaveConfiguration = new TalonFXConfiguration();

        rightMasterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMasterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftSlaveConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftSlaveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightMaster.getConfigurator().apply(rightMasterConfiguration);
        leftSlave.getConfigurator().apply(leftSlaveConfiguration);
       
        leftSlave.setControl(new Follower(rightMaster.getDeviceID(), false));

        // Reset
        stopControllers();
        resetSensors();
    }

    /**
     * Misc Stuff
     */
    public void setMotorOutput(double percent) {
        motorOut.Output = percent;
        rightMaster.setControl(motorOut);
        //rightMaster.set(1);
    }

    public void selectProfileSlot(int slot) {
        //motorOut.selectProfileSlot(slot, 0);
    }

    public void stopControllers() {
        motorOut.Output = 0;
        rightMaster.setControl(motorOut);
    }

    // Sensor Reset
    public void resetSensors() {
        //motorOut.setSelectedSensorPosition(0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}
