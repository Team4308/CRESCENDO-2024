// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
 
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.LEDSystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.PixySystem;
import frc.robot.subsystems.RotateShooterSystem;
import frc.robot.subsystems.pigeon2System;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve")); 
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();
  
  //Subsystems
  private final IntakeSystem m_intakeSystem;
  private final LEDSystem m_ledSystem;
  private final PixySystem pixy;
  private final RotateShooterSystem m_rotateShooterSystem;
  private final pigeon2System m_pigeon2System;

  //Commands
  private final IntakeCommand intakeCommand;
  private final LEDCommand ledCommand;
  private final RotateShooterCommand rotateShooterCommand;

  // Controllers
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final XBoxWrapper stick = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  public double shooterDegree = 20.0;

  //state machines
  private boolean shooterAutonTriggered = false;
  
  //Auto
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();
  

  // led stuff
  private Integer debounce = 0;
  private Double prev = 0.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    //Subsystem Instantiations
    m_intakeSystem = new IntakeSystem();
    subsystems.add(m_intakeSystem);

    pixy = new PixySystem();
    subsystems.add(pixy);

    m_ledSystem = new LEDSystem();
    subsystems.add(m_ledSystem);

    m_rotateShooterSystem = new RotateShooterSystem();
    subsystems.add(m_rotateShooterSystem);

    m_pigeon2System = new pigeon2System();
    subsystems.add(m_pigeon2System);
    
    //Command Instantiations
    intakeCommand = new IntakeCommand(m_intakeSystem, () -> 0.0);
    m_intakeSystem.setDefaultCommand(intakeCommand);
    
    ledCommand = new LEDCommand(m_ledSystem, () -> getLEDCommand());
    m_ledSystem.setDefaultCommand(ledCommand);

    rotateShooterCommand = new RotateShooterCommand(m_rotateShooterSystem, getRotateShooterControl());
    m_rotateShooterSystem.setDefaultCommand(rotateShooterCommand);

    SmartDashboard.putData(autoCommandChooser);
    
    // Configure the trigger bindings
    
    configureBindings();
    
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings()
  { 
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              )); 
    driverXbox.x().whileTrue(Commands.deferredProxy(() -> drivebase.aimAtTarget()));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    stick.Y.whileTrue(new IntakeCommand(m_intakeSystem, () -> getIntakeControl()));
    stick.LB.onTrue(new InstantCommand(() -> drivebase.alignToNote(true)));
    stick.LB.onFalse(new InstantCommand(() -> drivebase.alignToNote(false)));
    
    stick2.B.onTrue(new InstantCommand(() -> drivebase.alignToSpeaker(true)));
    stick2.B.onFalse(new InstantCommand(() -> drivebase.alignToSpeaker(false)));
    stick2.B.whileTrue(new InstantCommand(() -> m_rotateShooterSystem.autoAlignShooter()));
    stick2.B.onTrue(new InstantCommand(() -> setShooterAutonTriggered(true)));
    stick2.B.onFalse(new InstantCommand(() -> setShooterAutonTriggered(false)));

    stick2.A.onTrue(new InstantCommand(() -> m_rotateShooterSystem.resetSensors()));//debugging
  }

  public Command getAutonomousCommand()
  {
    return autoCommandChooser.getSelected();
  }

  public Command setShooterAutonTriggered(boolean value) {
    shooterAutonTriggered = value;
    return null;
  }

  public double getRotateShooterControl(){
    if (shooterAutonTriggered == false) {
      var newVal = stick.getRightY();
      if (-0.6 <= newVal && newVal <= 0.6) {//deadband; too lazy to code properly
        newVal = 0;
      }
      double newShooterDegree = shooterDegree + newVal;
      if (16 <= newShooterDegree && newShooterDegree <= 43) {//could use more fine tuning
        shooterDegree = newShooterDegree;
      }
    }
    
    return shooterDegree;
  }

  public Double getLEDCommand(){
    
    if(RobotController.isBrownedOut()){
      prev = 0.63;
      return 0.63; // red-orange
    }
    if(pixy.getClosestTarget() != null){
      // target in range
      debounce++;
      if(debounce == 5) debounce = 0;
      prev = -0.09;
      return -0.09;
    }
    if(pixy.getClosestTarget() == null){
      debounce--;
      if(debounce <= 0) debounce = 0;
    }
    if(debounce == 0){
      prev = -0.39;
      return -0.39;
    }  // default enabled, colour waves lava

    return prev;
    // disabled state is slow rgb
  }

  public double getIntakeControl() {
    return 1.0;
  }

  public void setDriveMode()
  {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake); 
  }
}