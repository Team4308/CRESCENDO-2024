package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;


public class ShooterSubsystem extends LogSubsystem {
    public final TalonFX right;
    public final TalonFX left;

    private final DutyCycleOut rightMotorOut;
    private final DutyCycleOut leftMotorOut;

    private final TalonFXConfiguration rightConfiguration;
    private final TalonFXConfiguration leftConfiguration;

    public ShooterSubsystem() {
        // Create Motor Controller Objects
        right = new TalonFX(Constants.Mapping.ShooterMotor.kMotor1);
        left = new TalonFX(Constants.Mapping.ShooterMotor.kMotor2);

        rightMotorOut = new DutyCycleOut(0);
        leftMotorOut = new DutyCycleOut(0);

        rightConfiguration = new TalonFXConfiguration();
        leftConfiguration = new TalonFXConfiguration();

        rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        right.getConfigurator().apply(rightConfiguration);
        left.getConfigurator().apply(leftConfiguration);
       
        // Reset
        stopControllers();
        resetSensors();
    }

    /**
     * Misc Stuff
     */
    public void setMotorOutput(double percent) {
        rightMotorOut.Output = percent * Constants.Shooter.rightMultiplier;
        leftMotorOut.Output = percent * Constants.Shooter.leftMultipler;
        right.setControl(rightMotorOut);
        left.setControl(leftMotorOut);
        //right.set(1);
    }

    public void selectProfileSlot(int slot) {
        //motorOut.selectProfileSlot(slot, 0);
    }

    public void stopControllers() {
        rightMotorOut.Output = 0;
        leftMotorOut.Output = 0;
        right.setControl(rightMotorOut);
        right.setControl(leftMotorOut);
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
