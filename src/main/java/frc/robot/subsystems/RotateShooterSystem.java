// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class RotateShooterSystem extends LogSubsystem {
    public final TalonFX motor;
    //public final ProfiledPIDController pidController;
    public final PIDController pidController;
    //public final ArmFeedforward feedforward;
    public final DigitalInput limitSwitch1;
    public final DigitalInput limitSwitch2;
    private final DutyCycleEncoder revEncoder; 

    public static double shooterDegree;
    public static double encoderDegree;
    public static double offset;
    public static double x = 0.0;

    public RotateShooterSystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
        
        revEncoder = new DutyCycleEncoder(Constants.Mapping.Shooter.encoder);

        limitSwitch1 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch1);
        limitSwitch2 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch2);

        motor.setNeutralMode(NeutralModeValue.Brake);

        // pidController = new ProfiledPIDController(
        //                     Constants.Shooter.AngleControl.kP, 
        //                     Constants.Shooter.AngleControl.kI, 
        //                     Constants.Shooter.AngleControl.kD,
        //                     new TrapezoidProfile.Constraints(Constants.Shooter.TrapezoidProfile.kMaxVelocity, 
        //                                                      Constants.Shooter.TrapezoidProfile.kMaxAcceleration));
        // feedforward = new ArmFeedforward(Constants.Shooter.FeedforwardControl.kS, Constants.Shooter.FeedforwardControl.kG, Constants.Shooter.FeedforwardControl.kV);

        pidController = new PIDController(Constants.Shooter.AngleControl.kP, 
                                          Constants.Shooter.AngleControl.kI, 
                                          Constants.Shooter.AngleControl.kD);
    }

    public void setMotorOutput(double percent){
        motor.set(percent);
    }

    public double getMotorPosition() {
        if (limitSwitch1.get()){
            revEncoder.reset();
            offset = Constants.Shooter.encoderStartRevolutions;
        } else if (limitSwitch2.get()) {
            revEncoder.reset();
            offset = Constants.Shooter.encoderEndRevolutions;
        }

        return revEncoder.getDistance() + offset;
    }

    public void setMotorPosition(double degree) { 
        SmartDashboard.putNumber("encoderDegree", revEncoder.getDistance());
        
        double wantedDegree = DoubleUtils.clamp(degree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        double shooterDegree = DoubleUtils.mapRangeNew(getMotorPosition(), Constants.Shooter.encoderStartRevolutions, Constants.Shooter.encoderEndRevolutions, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        // double motorOutput = -DoubleUtils.clamp(pidController.calculate(shooterDegree, wantedDegree) + feedforward.calculate(Math.toRadians(wantedDegree), pidController.getSetpoint().velocity), -1.0, 1.0);
        double motorOutput = -DoubleUtils.clamp(pidController.calculate(shooterDegree, wantedDegree) + Math.cos(wantedDegree) * Constants.Shooter.FeedforwardControl.kG, -1.0, 1.0);
        
        SmartDashboard.putNumber("shooterDegree", shooterDegree);
        SmartDashboard.putNumber("wantedDegree", wantedDegree);
        SmartDashboard.putNumber("motorOutput", motorOutput);

        setMotorOutput(motorOutput);
    }

    public Double autoAlignShooter() {
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("");

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = Constants.Limelight.Measurements.limelightMountAngleDegrees; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = Constants.Limelight.Measurements.limelightLensHeightCM;

        // distance from the target to the floor
        double goalHeightCM = Constants.GamePieces.speaker.speakerAprilTagHeightCM + x;
        double speakerOpeningHeightCM = Constants.GamePieces.speaker.speakerOpeningHeightCM;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double limelightDistanceFromShooterCM = Constants.Limelight.Measurements.limelightDistanceFromShooterCM;

        //calculate distance
        double distanceFromShooterToGoalCM = (goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians) + limelightDistanceFromShooterCM;

        double shooterAngle = Math.atan(speakerOpeningHeightCM/distanceFromShooterToGoalCM) * (180.0 / 3.14159);

        return shooterAngle;
    }

    public void changeGoalHeight(Double value) {
        x = x + value;
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
