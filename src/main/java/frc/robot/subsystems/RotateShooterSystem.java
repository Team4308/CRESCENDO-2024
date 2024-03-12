// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class RotateShooterSystem extends LogSubsystem {
    public final TalonFX motor;
    private final TalonFXConfiguration motorConfiguration;
    public final PIDController pidController;
    public final DigitalInput limitSwitch1;
    public final DigitalInput limitSwitch2;
    private final DutyCycleEncoder revEncoder; 
    private final Pigeon2 gyro = new Pigeon2(Constants.Mapping.Pigeon2.gyro);

    public static double shooterDegree;
    public static double encoderDegree;

    public RotateShooterSystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
        
        revEncoder = new DutyCycleEncoder(Constants.Mapping.Shooter.encoder);
        revEncoder.setDistancePerRotation(1.0);
        revEncoder.reset();

        limitSwitch1 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch1);
        limitSwitch2 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch2);

        motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(motorConfiguration);

        pidController = new PIDController(Constants.Shooter.AngleControl.kP, Constants.Shooter.AngleControl.kI, Constants.Shooter.AngleControl.kD);//pid not tuned
    }

    public void setMotorOutput(double percent){
        motor.set(percent);
    }

    public double getMotorPosition() {
        if (limitSwitch1.get()){
            revEncoder.reset();
        } else if (limitSwitch2.get()) {
            return Constants.Shooter.encoderEndRevolutions;
        }
        return revEncoder.getDistance();
    }

    public void setMotorPosition(double degree) { 
        double wantedDegree = DoubleUtils.clamp(degree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        double outputDegree = DoubleUtils.mapRangeNew(wantedDegree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree, Constants.Shooter.encoderStartRevolutions, Constants.Shooter.encoderEndRevolutions);

        double encoderDegree = getMotorPosition();

        SmartDashboard.putNumber("Shooter Angle", DoubleUtils.mapRangeNew(encoderDegree, Constants.Shooter.encoderStartRevolutions, Constants.Shooter.encoderEndRevolutions, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree));

        SmartDashboard.putNumber("encoder", encoderDegree);

        double motorOutput = DoubleUtils.clamp(pidController.calculate(encoderDegree, outputDegree), -1.0, 1.0);

        setMotorOutput(motorOutput);
    }

    public Double autoAlignShooter() {
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("");

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = Constants.Limelight.Measurements.limelightMountAngleDegrees; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = Constants.Limelight.Measurements.limelightLensHeightCM;

        // distance from the target to the floor
        double goalHeightCM = Constants.GamePieces.speaker.speakerAprilTagHeightCM;
        double speakerOpeningHeightCM = Constants.GamePieces.speaker.speakerOpeningHeightCM;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double limelightDistanceFromShooterCM = Constants.Limelight.Measurements.limelightDistanceFromShooterCM;

        //calculate distance
        double distanceFromShooterToGoalCM = (goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians) + limelightDistanceFromShooterCM;

        double accelY = gyro.getAccelerationY().getValueAsDouble();
        double offset = accelY * 0.0;   // shootingwhilemoving thing

        double shooterAngle = Math.atan(speakerOpeningHeightCM/distanceFromShooterToGoalCM) * (180.0 / 3.14159) + offset;

        SmartDashboard.putNumber("autoalign", shooterAngle);

        return shooterAngle;
    }

    public void controlWithController(Double controllerValue) {
        shooterDegree = DoubleUtils.clamp(shooterDegree + controllerValue, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);
        setMotorPosition(shooterDegree);
    }

    public void resetSensors() {
        motor.setPosition(0);
    }

    public void stopControllers(){
        motor.set(0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}
