// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import ca.team4308.absolutelib.wrapper.LoggedTunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class ArmSubsystem extends LogSubsystem {
    private final TalonFX motor;
    private final PIDController pidController;
    private ArmFeedforward feedforward;
    private final DutyCycleEncoder encoder; 
    public final DigitalInput lowerLimitSwitch;
    public final DigitalInput upperLimitSwitch;
    

    private LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", Constants.Shooter.AngleControl.kP);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", Constants.Shooter.AngleControl.kI);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", Constants.Shooter.AngleControl.kD);
    private LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", Constants.Shooter.FeedforwardControl.kS);
    private LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", Constants.Shooter.FeedforwardControl.kV);
    private LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", Constants.Shooter.FeedforwardControl.kA);
    private LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", Constants.Shooter.FeedforwardControl.kG);

    public ArmSubsystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
        encoder = new DutyCycleEncoder(Constants.Mapping.Shooter.encoder);
        lowerLimitSwitch = new DigitalInput(Constants.Mapping.Shooter.limitSwitch1);
        upperLimitSwitch = new DigitalInput(Constants.Mapping.Shooter.limitSwitch2);

        pidController = new PIDController(kP.get(), kI.get(), kD.get());
        feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setMotorOutput(double percent){
        motor.set(percent);
    }

    public double getMotorPosition() {
        if (!lowerLimitSwitch.get()){
            encoder.reset();
            offset = Constants.Shooter.encoderStartRevolutions;
        } else if (upperLimitSwitch.get()) {
            encoder.reset();
            offset = Constants.Shooter.encoderEndRevolutions;
        }

        return encoder.getDistance() + offset;
    }

    public void setMotorPosition(double degree) { 
        wantedDegree = DoubleUtils.clamp(degree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        shooterDegree = DoubleUtils.mapRangeNew(getMotorPosition(), Constants.Shooter.encoderStartRevolutions, Constants.Shooter.encoderEndRevolutions, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);
        // This code is wrong, needs to be fixed
        motorOutput = -DoubleUtils.clamp(pidController.calculate(shooterDegree, wantedDegree), -1.0, 1.0);

        setMotorOutput(motorOutput);
    }

    public double alignDistaceToSpeaker(double distanceToSpeaker) {
        double speakerOpeningHeight = Constants.GamePieces.Speaker.kSpeakerCenterBlue.getZ();
        double shooterAngle = Math.atan(speakerOpeningHeight/distanceToSpeaker)  * (180.0 / 3.14159);
        return shooterAngle;
    }

    public Double autoAlignShooter() {
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("");

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = Constants.Limelight.Measurements.limelightMountAngleDegrees; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = Constants.Limelight.Measurements.limelightLensHeightCM;

        // distance from the target to the floor
        double goalHeightCM = Constants.GamePieces.Speaker.speakerAprilTagHeightCM + x;
        double speakerOpeningHeightCM = Constants.GamePieces.Speaker.speakerOpeningHeightCM;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double limelightDistanceFromShooterCM = Constants.Limelight.Measurements.limelightDistanceFromShooterCM;

        //calculate distance
        double distanceFromShooterToGoalCM = (goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians) + limelightDistanceFromShooterCM;

        double shooterAngle = Math.atan(speakerOpeningHeightCM/distanceFromShooterToGoalCM) * (180.0 / 3.14159);

        return shooterAngle;
    }

    // Only updates values when the robot is enabled
    public void periodic() {
        checkTunableValues();
    }

    public void checkTunableValues() {
        if (!Constants.LoggedDashboard.tuningMode) {
            return;
        }
        if (DriverStation.isEnabled()) {
            LoggedTunableNumber.ifChanged(
                hashCode(), () -> pidController.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
            LoggedTunableNumber.ifChanged(
                hashCode(), () -> feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
            kS, kG, kV, kA);
        }
    }

    public void changeGoalHeight(Double value) {
        x = x + value;
    }

    // public void controlWithController(Double controllerValue) {
    //     shooterDegree = DoubleUtils.clamp(shooterDegree + controllerValue, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);
    //     setMotorPosition(shooterDegree);
    // }

    public void resetSensors() {
        motor.setPosition(0);
    }

    public void stopControllers(){
        motor.set(0);
    }

    public void setShooterAutonTriggered(boolean value) {
        shooterAutonTriggered = value;
    }

    public boolean getShooterAutonTriggered() {
        return shooterAutonTriggered;
    }

    public Command setShooterAutonCommand(boolean value) {
        return this.runOnce(() -> setShooterAutonTriggered(value));
    }

    @Override
    public Sendable log() {
        SmartDashboard.putBoolean("Pivot/lowerLimitSwitch1", !lowerLimitSwitch.get());
        SmartDashboard.putNumber("Pivot/rawEncoderDegree", encoder.getDistance());
        SmartDashboard.putNumber("Pivot/updatedEncoderDegree", getMotorPosition());
        SmartDashboard.putNumber("Pivot/shooterDegree", shooterDegree);
        SmartDashboard.putNumber("Pivot/wantedDegree", wantedDegree);
        SmartDashboard.putNumber("Pivot/motorOutput", motorOutput);
        return this;
    }
}