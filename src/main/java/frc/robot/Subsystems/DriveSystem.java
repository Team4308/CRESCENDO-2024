package frc.robot.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.drive.TankDriveSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class DriveSystem extends TankDriveSubsystem {
        // Master Controllers
        public final TalonSRX masterLeft, masterRight;

        // Slave Controllers
        private final TalonSRX slaveLeft, slaveRight;

        // Controllers
        private ArrayList<TalonSRX> controllersSRX = new ArrayList<TalonSRX>();

        // Init
        public DriveSystem() {
                // Setup and Add Controllers
                masterLeft = new TalonSRX(Constants.Mapping.Drive.frontLeft);
                controllersSRX.add(masterLeft);
                masterRight = new TalonSRX(Constants.Mapping.Drive.frontRight);
                controllersSRX.add(masterRight);
                slaveLeft = new TalonSRX(Constants.Mapping.Drive.backLeft);
                controllersSRX.add(slaveLeft);
                slaveRight = new TalonSRX(Constants.Mapping.Drive.backRight);
                controllersSRX.add(slaveRight);
                
                // Reset Config for all
                for (TalonSRX talon : controllersSRX) {
                        talon.configFactoryDefault(Constants.Generic.timeoutMs);
                }

                // Set Invert Mode
                masterLeft.setInverted(false);
                masterRight.setInverted(true);
                slaveLeft.setInverted(false);
                slaveRight.setInverted(false);

                // Change Config For All Controllers
                for (TalonSRX talon : controllersSRX) {
                        talon.configFactoryDefault(Constants.Generic.timeoutMs);
                        talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp,
                                        Constants.Generic.timeoutMs);
                        talon.configClosedloopRamp(Constants.Config.Drive.Power.kClosedLoopRamp,
                                        Constants.Generic.timeoutMs);
                        talon.setNeutralMode(NeutralMode.Coast);
                        talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
                        talon.changeMotionControlFramePeriod(5);
                        talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
                        talon.enableVoltageCompensation(true);
                }

                // Set Sensor Phase for all loops
                masterLeft.setSensorPhase(false);
                // masterRight.setSensorPhase(false);

                // Set Left Velocity PIDF values
                masterLeft.config_kP(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Left.kP, Constants.Generic.timeoutMs);
                masterLeft.config_kI(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Left.kI, Constants.Generic.timeoutMs);
                masterLeft.config_kD(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Left.kD, Constants.Generic.timeoutMs);
                masterLeft.config_kF(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Left.kF, Constants.Generic.timeoutMs);
                masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);

                // Set Right Velocity PIDF values
                masterRight.config_kP(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Right.kP, Constants.Generic.timeoutMs);
                masterRight.config_kI(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Right.kI, Constants.Generic.timeoutMs);
                masterRight.config_kD(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Right.kD, Constants.Generic.timeoutMs);
                masterRight.config_kF(Constants.Config.Drive.VelocityControl.profileSlot,
                                Constants.Config.Drive.VelocityControl.Right.kF, Constants.Generic.timeoutMs);
                masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);

                masterLeft.configMotionCruiseVelocity(Constants.Config.Drive.MotionMagic.maxVel);
                masterRight.configMotionCruiseVelocity(Constants.Config.Drive.MotionMagic.maxVel);
                masterLeft.configMotionAcceleration(Constants.Config.Drive.MotionMagic.maxAcc);
                masterRight.configMotionAcceleration(Constants.Config.Drive.MotionMagic.maxAcc);

                masterLeft.config_kP(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kP, Constants.Generic.timeoutMs);
                masterLeft.config_kI(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kI, Constants.Generic.timeoutMs);
                masterLeft.config_kD(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kD, Constants.Generic.timeoutMs);
                masterLeft.config_kF(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kF, Constants.Generic.timeoutMs);
                masterLeft.config_IntegralZone(Constants.Config.Drive.MotionMagic.profileSlot, 10);
                masterRight.config_kP(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kP, Constants.Generic.timeoutMs);
                masterRight.config_kI(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kI, Constants.Generic.timeoutMs);
                masterRight.config_kD(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kD, Constants.Generic.timeoutMs);
                masterRight.config_kF(Constants.Config.Drive.MotionMagic.profileSlot,
                                Constants.Config.Drive.MotionMagic.Right.kF, Constants.Generic.timeoutMs);
                masterRight.config_IntegralZone(Constants.Config.Drive.MotionMagic.profileSlot, 10);

                // Reset
                stopControllers();
                resetSensors();
        }

        /**
         * Getters And Setters
         */
        public double getLeftSensorPosition() {
                return masterLeft.getSelectedSensorPosition(0);
                               // * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
        }

        public double getRightSensorPosition() {
                return masterRight.getSelectedSensorPosition(0)
                                * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
        }

        public double getLeftSensorVelocity() {
                return masterLeft.getSelectedSensorVelocity(0);
        }

        public double getRightSensorVelocity() {
                return masterRight.getSelectedSensorVelocity(0);
        }

        
        public void setMotorOutput(ControlMode mode, double left, double right) {
                masterLeft.set(mode, left);
                masterRight.set(mode, right);
                slaveLeft.set(mode, left);
                slaveRight.set(mode, right);
        }

        public void selectProfileSlot(int slot) {
                masterLeft.selectProfileSlot(slot, 0);
                masterRight.selectProfileSlot(slot, 0);
        }

        public void resetAngle() {
        }

        public void stopControllers() {
                masterLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
                masterRight.set(TalonSRXControlMode.PercentOutput, 0.0);
        }

        // Sensor Reset
        public void resetSensors() {
                masterLeft.setSelectedSensorPosition(0);
                masterRight.setSelectedSensorPosition(0);
        }

        @Override
        public Sendable log() {
                return this;
        }
}