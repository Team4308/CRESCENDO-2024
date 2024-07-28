package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class ShooterSubsystem extends LogSubsystem {
    public final TalonFX right;
    public final TalonFX left;

    private final DutyCycleOut rightMotorOut;
    private final DutyCycleOut leftMotorOut;

    private final TalonFXConfiguration rightConfiguration;
    private final TalonFXConfiguration leftConfiguration;

    final VelocityVoltage rightVelocity;
    final VelocityVoltage leftVelocity;
    
    public double maxSpeed;

    public double bottomMultiplier;
    public double topMultiplier;

    public ShooterSubsystem() {
        // Create Motor Controller Objects
        right = new TalonFX(Constants.Mapping.ShooterMotor.kMotor1);
        left = new TalonFX(Constants.Mapping.ShooterMotor.kMotor2);

        rightMotorOut = new DutyCycleOut(0);
        leftMotorOut = new DutyCycleOut(0);

        rightVelocity = new VelocityVoltage(0);
        leftVelocity = new VelocityVoltage(0);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = Constants.Shooter.ShooterControl.kV;
        slot0Configs.kP = Constants.Shooter.ShooterControl.kP;
        slot0Configs.kI = Constants.Shooter.ShooterControl.kI;
        slot0Configs.kD = Constants.Shooter.ShooterControl.kD;

        rightConfiguration = new TalonFXConfiguration();
        leftConfiguration = new TalonFXConfiguration();

        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        right.getConfigurator().apply(rightConfiguration);
        left.getConfigurator().apply(leftConfiguration);

        right.getConfigurator().apply(slot0Configs, Constants.Generic.timeoutMs);
        left.getConfigurator().apply(slot0Configs, Constants.Generic.timeoutMs);

        maxSpeed = 100;
        topMultiplier = 1;
        bottomMultiplier = 1;
        
        // Reset
        stopControllers();
        resetSensors();
    }

    public void changeTopMultiplier(double newValue) {
        topMultiplier = newValue;
    }

    public void changeBottomMultiplier(double newValue) {
        bottomMultiplier = newValue;
    }

    public Command changeAmpTopMultiplierCommand() {
        return this.startEnd(() -> changeTopMultiplier(Constants.Shooter.shootInAmpMultiplier), () -> changeTopMultiplier(1.0));
    }

    /**
     * Misc Stuff
     */
    public void setMotorOutput(double rps) {
        rightVelocity.Slot = 0;
        leftVelocity.Slot = 0;
        right.setControl(rightVelocity.withVelocity(rps*topMultiplier));
        left.setControl(leftVelocity.withVelocity(rps*bottomMultiplier));

        /*
        rightMotorOut.Output = rpm * Constants.Shooter.rightMultiplier;
        leftMotorOut.Output = rpm * Constants.Shooter.leftMultipler;
        right.setControl(rightMotorOut);
        left.setControl(leftMotorOut);
        //right.set(1);
        */
    }

    public void setMaxSpeed(double rps) {
        maxSpeed = rps;
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
        right.getConfigurator().setPosition(0);
        left.getConfigurator().setPosition(0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}
