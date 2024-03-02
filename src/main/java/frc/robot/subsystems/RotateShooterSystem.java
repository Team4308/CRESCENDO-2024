// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;


public class RotateShooterSystem extends SubsystemBase {
    public final TalonFX motor;

    public RotateShooterSystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
    }

    public void setMotorOutput(double percent){
        motor.set(percent);
    }

    public double getMotorPosition() {
        var rotorPosSignal = motor.getRotorPosition();
        var rotorPos = rotorPosSignal.getValue();

        return rotorPos;
    }
    public void resetSensors() {
        motor.setPosition(0);
    }

    public void stopControllers(){
        motor.set(0);
    }
}
