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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class PivotSubsystem extends LogSubsystem {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/PID/kP", Constants.Shooter.PivotPID.kP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/PID/kI", Constants.Shooter.PivotPID.kI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/PID/kD", Constants.Shooter.PivotPID.kD);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/FF/kS", Constants.Shooter.PivotFF.kS);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/FF/kV", Constants.Shooter.PivotFF.kV);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/FF/kA", Constants.Shooter.PivotFF.kA);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/FF/kG", Constants.Shooter.PivotFF.kG);
    private static final LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Pivot/MaxVel", Constants.Shooter.TrapezoidProfile.kMaxVelocity);
    private static final LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("Pivot/MaxAccel", Constants.Shooter.TrapezoidProfile.kMaxAcceleration);

    private final TalonFX motor = new TalonFX(Constants.Mapping.Shooter.motor);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Mapping.Shooter.encoder);    
    public final DigitalInput lowerLimitSwitch = new DigitalInput(Constants.Mapping.Shooter.limitSwitch1);
    public final DigitalInput upperLimitSwitch = new DigitalInput(Constants.Mapping.Shooter.limitSwitch2);

    private final PIDController pidController = new PIDController(kP.get(), kI.get(), kD.get());
    private ArmFeedforward feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    public TrapezoidProfile profiler = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    private double desiredSetpoint = 0;
    private double desiredVelocity = 0;
    private double offset;

    public PivotSubsystem() {
        motor.setNeutralMode(NeutralModeValue.Coast);
        motor.setInverted(true);
        encoder.reset();
        changeSetpoint(getCurrentPosition());
    }

    public boolean lowerLimitSwitchIsTrue() {
        return lowerLimitSwitch.get();
    }

    public boolean upperLimitSwitchIsTrue() {
        return upperLimitSwitch.get();
    }

    public double getCurrentPosition() {
        if (lowerLimitSwitchIsTrue()) {
            encoder.reset();
            offset = Constants.Shooter.encoderStartRevolutions;
        } else if (upperLimitSwitchIsTrue()) {
            encoder.reset();
            offset = Constants.Shooter.encoderEndRevolutions;
        }
        double encoderValue = encoder.getDistance() + offset;
        double shooterDegree = DoubleUtils.mapRangeNew(encoderValue, 
                               Constants.Shooter.encoderStartRevolutions, Constants.Shooter.encoderEndRevolutions, 
                               Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);
        return shooterDegree;
    }

    public double getDesiredSetpoint() {
        return desiredSetpoint;
    }
    
    // Clamp values so that the desired setpoint doesnt exceed min or max angles
    public void changeSetpoint(double newSetpoint) {
        if (newSetpoint < Constants.Shooter.shooterStartDegree) {
            this.desiredSetpoint = Constants.Shooter.shooterStartDegree;
        } else if (newSetpoint > Constants.Shooter.shooterEndDegree) {
            this.desiredSetpoint = Constants.Shooter.shooterEndDegree;
        } else {
            this.desiredSetpoint = newSetpoint;
        }
    }

    public void changeVelocity(double newVelocity) {
        this.desiredVelocity = newVelocity;
    }

    public void changeState(TrapezoidProfile.State state) {
        changeSetpoint(state.position);
        changeVelocity(state.velocity);
    }

    public TrapezoidProfile.State getCurrentState() {
        return new TrapezoidProfile.State(getCurrentPosition(), this.desiredVelocity);
    }

    public double getShooterAngleToSpeaker(double distanceToSpeaker) {
        double offset = 0.0;
        double speakerOpeningHeight = Constants.GamePieces.Speaker.kSpeakerCenterBlue.getZ() + offset;
        double shooterAngle = Math.atan(speakerOpeningHeight/distanceToSpeaker)  * (180.0 / 3.14159);
        return shooterAngle;
    }

    public double getMotorOutput() {
        double outputFF = feedforward.calculate(desiredSetpoint * (Math.PI/180), desiredVelocity * (Math.PI/180));
        double outputPID = pidController.calculate(getCurrentPosition() * (Math.PI/180), desiredSetpoint * (Math.PI/180));
        return outputFF + outputPID;
    }

    public void setMotorOutput(){
        motor.setVoltage(getMotorOutput());
    }

    public void periodic() {
        checkTunableValues();
        setMotorOutput();
        log();
    }

    public void checkTunableValues() {
        if (!Constants.LoggedDashboard.tuningMode) {
            return;
        }
        // Only update LoggedTunableNumbers when enabled
        if (DriverStation.isEnabled()) {
            LoggedTunableNumber.ifChanged(hashCode(), 
            () -> pidController.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
            LoggedTunableNumber.ifChanged(hashCode(), 
            () -> feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
            kS, kG, kV, kA);
        }
    }

    public void resetSensors() {
        motor.setPosition(0);
    }

    public void stopControllers(){
        motor.set(0);
    }
    
    @Override
    public Sendable log() {
        SmartDashboard.putBoolean("Pivot/Lower Limit Switch", lowerLimitSwitchIsTrue());
        SmartDashboard.putBoolean("Pivot/Upper Limit Switch", upperLimitSwitchIsTrue());
        SmartDashboard.putNumber("Pivot/Raw Encoder Value", encoder.getDistance());
        SmartDashboard.putNumber("Pivot/Current Setpoint", getCurrentPosition());
        SmartDashboard.putNumber("Pivot/Desired Setpoint", desiredSetpoint);
        SmartDashboard.putNumber("Pivot/Desired Velocity", desiredVelocity);
        SmartDashboard.putNumber("Pivot/Motor Output", getMotorOutput());
        return this;
    }
}