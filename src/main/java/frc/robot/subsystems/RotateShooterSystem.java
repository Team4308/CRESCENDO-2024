// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.sound.sampled.ReverbType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    public final DigitalInput limitSwitch;
    private final DutyCycleEncoder revEncoder;

    public static double shooterDegree = 20.0;

    public static double encoderDegree = 0.0;

    public RotateShooterSystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
        revEncoder = new DutyCycleEncoder(Constants.Mapping.encoder.encoder);
        limitSwitch = new DigitalInput(Constants.Mapping.limitSwitch.limitSwitch);

        motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motor.getConfigurator().apply(motorConfiguration);

        pidController = new PIDController(Constants.Shooter.AngleControl.kP, Constants.Shooter.AngleControl.kI, Constants.Shooter.AngleControl.kD);//pid not tuned
    }

    public void setMotorOutput(double percent){
        motor.set(percent);
    }

    public double getMotorPosition() {
        if (limitSwitch.get()){
            revEncoder.reset();
        }
        return revEncoder.getDistance();
    }

    public void setMotorPosition(double degree) { 
        double wantedDegree = DoubleUtils.clamp(degree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        /*
        double m = (Constants.Shooter.motorStartRevolutions-Constants.Shooter.motorEndRevolutions)/(Constants.Shooter.shooterStartDegree-Constants.Shooter.shooterEndDegree);
        double b = Constants.Shooter.motorStartRevolutions-(Constants.Shooter.shooterStartDegree*m);
        double outputDegree = m*wantedDegree+b;
        */

        double outputDegree = DoubleUtils.mapRangeNew(wantedDegree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree, Constants.Shooter.motorStartRevolutions, Constants.Shooter.motorEndRevolutions);

        double encoderDegree = getMotorPosition();

        SmartDashboard.putNumber("Shooter Angle", DoubleUtils.mapRangeNew(encoderDegree, Constants.Shooter.motorStartRevolutions, Constants.Shooter.motorEndRevolutions, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree));

        double motorDegree = DoubleUtils.clamp(pidController.calculate(encoderDegree, outputDegree), -1.0, 1.0);

        setMotorOutput(motorDegree);
    }

    public void autoAlignShooter() {
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("");

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = Constants.Limelight.Measurements.limelightMountAngleDegrees; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = Constants.Limelight.Measurements.limelightLensHeightCM;

        // distance from the target to the floor
        double goalHeightCM = Constants.GamePieces.Dimensions.stageHeightCM;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromShooterToGoalCM = (goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians) + 48.26;

        double shooterAngle = Math.atan(goalHeightCM/distanceFromShooterToGoalCM);

        setMotorPosition(shooterAngle);
    }

    public void controlWithController(Double controllerValue) {
        double newShooterDegree = shooterDegree + controllerValue;
        if (Constants.Shooter.shooterStartDegree <= newShooterDegree && newShooterDegree <= Constants.Shooter.shooterEndDegree) {//could use more fine tuning
        shooterDegree = newShooterDegree;
        }
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
