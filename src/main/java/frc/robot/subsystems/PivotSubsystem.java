// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import ca.team4308.absolutelib.wrapper.LoggedTunableNumber;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class PivotSubsystem extends LogSubsystem {
    private final TalonFX motor;
    private final PIDController pidController;
    //public final PIDController pidController;
    private ArmFeedforward feedforward;
    private final DutyCycleEncoder revEncoder; 
    public final DigitalInput limitSwitch1;
    public final DigitalInput limitSwitch2;
    
    public static double encoderDegree;
    public static double shooterDegree;
    public static double wantedDegree;
    public static double motorOutput;

    public static double offset;
    public static double x = 0.0;
    public static boolean shooterAutonTriggered = false;
    public double currentShooterDegree = Constants.Shooter.shooterStartDegree;
    private Debouncer debouncer;

    private static final LoggedTunableNumber kP = 
        new LoggedTunableNumber("Pivot/Gains/kP", Constants.Shooter.AngleControl.kP);
    private static final LoggedTunableNumber kI = 
        new LoggedTunableNumber("Pivot/Gains/kI", Constants.Shooter.AngleControl.kI);
    private static final LoggedTunableNumber kD = 
        new LoggedTunableNumber("Pivot/Gains/kD", Constants.Shooter.AngleControl.kD);
    private static final LoggedTunableNumber kS =
        new LoggedTunableNumber("Arm/Gains/kS", Constants.Shooter.FeedforwardControl.kS);
    private static final LoggedTunableNumber kV =
        new LoggedTunableNumber("Arm/Gains/kV", Constants.Shooter.FeedforwardControl.kV);
    private static final LoggedTunableNumber kA =
        new LoggedTunableNumber("Arm/Gains/kA", Constants.Shooter.FeedforwardControl.kA);
    private static final LoggedTunableNumber kG =
        new LoggedTunableNumber("Arm/Gains/kG", Constants.Shooter.FeedforwardControl.kG);
    private static final LoggedTunableNumber maxVelocity =
        new LoggedTunableNumber("Arm/Velocity", Constants.Shooter.TrapezoidProfile.kMaxVelocity);
    private static final LoggedTunableNumber maxAcceleration =
        new LoggedTunableNumber("Arm/Acceleration", Constants.Shooter.TrapezoidProfile.kMaxAcceleration);

    public PivotSubsystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
        
        revEncoder = new DutyCycleEncoder(Constants.Mapping.Shooter.encoder);

        limitSwitch1 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch1);
        limitSwitch2 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch2);

        debouncer = new Debouncer(0.1, DebounceType.kBoth);

        motor.setNeutralMode(NeutralModeValue.Coast);

        pidController = new PIDController(kP.get(), kI.get(), kD.get());
        feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        // pidController = new PIDController(Constants.Shooter.AngleControl.kP, 
        //                                   Constants.Shooter.AngleControl.kI, 
        //                                   Constants.Shooter.AngleControl.kD);
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
        wantedDegree = DoubleUtils.clamp(degree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        shooterDegree = DoubleUtils.mapRangeNew(getMotorPosition(), Constants.Shooter.encoderStartRevolutions, Constants.Shooter.encoderEndRevolutions, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);
        // This code is wrong, needs to be fixed
        motorOutput = -DoubleUtils.clamp(pidController.calculate(shooterDegree, wantedDegree), -1.0, 1.0);

        setMotorOutput(motorOutput);
    }

    public double alignShooterToSpeaker(double distanceToSpeaker) {
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
        if (DriverStation.isEnabled()) {
            LoggedTunableNumber.ifChanged(
                hashCode(), () -> pidController.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
            LoggedTunableNumber.ifChanged(
                hashCode(), () -> feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
            kS, kG, kV, kA);
        }
        log();
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
        SmartDashboard.putBoolean("pivot/lowerLimitSwitch1", limitSwitch1.get());
        SmartDashboard.putBoolean("pivot/upperlimitswitch", limitSwitch2.get());
        SmartDashboard.putNumber("pivot/rawEncoderDegree", revEncoder.getDistance());
        SmartDashboard.putNumber("pivot/updatedEncoderDegree", getMotorPosition());
        SmartDashboard.putNumber("pivot/shooterDegree", shooterDegree);
        SmartDashboard.putNumber("pivot/wantedDegree", wantedDegree);
        SmartDashboard.putNumber("pivot/motorOutput", motorOutput);
        return this;
    }
}