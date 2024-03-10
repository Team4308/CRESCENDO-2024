// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    public final DigitalInput limitSwitch1;
    public final DigitalInput limitSwitch2;
    private final DutyCycleEncoder revEncoder; 
    private final pigeon2System m_gyroSystem = new pigeon2System();

    public static double shooterDegree = 20.0;
    public static double encoderDegree = 0.0;

    public RotateShooterSystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
        revEncoder = new DutyCycleEncoder(Constants.Mapping.Shooter.encoder);
        limitSwitch1 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch1);
        limitSwitch2 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch2);

        motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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
            return 66.0;
        }
        return revEncoder.getDistance() + 16;
    }

    public void setMotorPosition(double degree) { 
        double wantedDegree = DoubleUtils.clamp(degree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);
        //0 is base position(16 degree)
        //14.25925757 is max revolutions(43 degree)
        //2200/12 gear ratio  

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
        double limelightMountAngleDegrees = Constants.Limelight.measurements.limelightMountAngleDegrees; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = Constants.Limelight.measurements.limelightLensHeightCM;

        // distance from the target to the floor
        double goalHeightCM = Constants.gamePieces.speaker.speakerAprilTagHeightCM;
        double speakerOpeningHeightCM = Constants.gamePieces.speaker.speakerOpeningHeightCM;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalCM = (goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians);

        double accelY = m_gyroSystem.getAccelerationY();
        double offset = accelY * 0.5;

        double shooterAngle = Math.atan(speakerOpeningHeightCM/distanceFromLimelightToGoalCM) + offset;

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
