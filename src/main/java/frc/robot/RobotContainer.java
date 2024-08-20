package frc.robot;

import java.io.File;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.BeambreakCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem.DriveMode;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexSystem;

public class RobotContainer {

  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  // Subsystems
  private final SwerveSubsystem drivebase;
  private final IntakeSystem m_intakeSubsystem;
  private final PivotSubsystem m_pivotSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ClimbSubsystem m_climbSubsystem;
  private final IndexSystem m_indexSystem;

  // Commands
  private final IntakeCommand intakeCommand;
  private final PivotCommand pivotCommand;
  private final ShooterCommand shooterCommand;
  private final ClimbCommand climbCommand;
  private final IndexCommand indexCommand;

  // Controllers
  private final XBoxWrapper driver = new XBoxWrapper(Constants.Mapping.Controllers.driver);
  private final XBoxWrapper operator = new XBoxWrapper(Constants.Mapping.Controllers.operator);

  // Auto
  private final SendableChooser<Command> autonomousChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Subsystem Instantiations
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    subsystems.add(drivebase);

    m_intakeSubsystem = new IntakeSystem();
    subsystems.add(m_intakeSubsystem);

    m_pivotSubsystem = new PivotSubsystem();
    subsystems.add(m_pivotSubsystem);

    m_shooterSubsystem = new ShooterSubsystem();
    subsystems.add(m_shooterSubsystem);

    m_climbSubsystem = new ClimbSubsystem();
    subsystems.add(m_climbSubsystem);

    m_indexSystem = new IndexSystem();
    subsystems.add(m_indexSystem);

    NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_intakeSubsystem, () -> -1.0));
    NamedCommands.registerCommand("IndexCommand", new IndexCommand(m_indexSystem, () -> -0.15));
    NamedCommands.registerCommand("IndexShootCommand", new IndexCommand(m_indexSystem, () -> -1.0));
    NamedCommands.registerCommand("ShooterCommand",
        new ShooterCommand(m_shooterSubsystem, () -> Constants.Shooter.shooterRPS));
    NamedCommands.registerCommand("ShooterStop", new ShooterCommand(m_shooterSubsystem, () -> 0.0));
    NamedCommands.registerCommand("ShooterDisruption", new ShooterCommand(m_shooterSubsystem, () -> 25.0));
    // NamedCommands.registerCommand("SpeakerAlignTrue", Commands.runOnce(() ->
    // drivebase.alignToSpeaker(true)));
    // NamedCommands.registerCommand("SpeakerAlignFalse", Commands.runOnce(() ->
    // drivebase.alignToSpeaker(false)));
    NamedCommands.registerCommand("ResetGyro", Commands.runOnce(drivebase::zeroGyro));
    NamedCommands.registerCommand("AutoAlignShooter",
        new PivotCommand(m_pivotSubsystem, () -> m_pivotSubsystem.autoAlignShooter()));
    NamedCommands.registerCommand("BeambreakCommand", new BeambreakCommand(() -> m_indexSystem.getBeambreak()));
    NamedCommands.registerCommand("SubwooferAngle",
        new PivotCommand(m_pivotSubsystem, () -> Constants.GamePieces.speaker.angle));
    NamedCommands.registerCommand("SubwooferSideAngle", new PivotCommand(m_pivotSubsystem, () -> 60.0));
    NamedCommands.registerCommand("AmpAngle",
        new PivotCommand(m_pivotSubsystem, () -> Constants.GamePieces.amp.angleToshoot));
    
    // Command Instantiations
    intakeCommand = new IntakeCommand(m_intakeSubsystem, () -> getIntakeControl());
    m_intakeSubsystem.setDefaultCommand(intakeCommand);

    pivotCommand = new PivotCommand(m_pivotSubsystem, () -> getPivotControl());
    m_pivotSubsystem.setDefaultCommand(pivotCommand);

    shooterCommand = new ShooterCommand(m_shooterSubsystem, () -> getShooterControl());
    m_shooterSubsystem.setDefaultCommand(shooterCommand);

    climbCommand = new ClimbCommand(m_climbSubsystem, () -> 0.0);
    m_climbSubsystem.setDefaultCommand(climbCommand);

    indexCommand = new IndexCommand(m_indexSystem, () -> getIndexControl());
    m_indexSystem.setDefaultCommand(indexCommand);

    autonomousChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autonomousChooser);

    drivebase.setSpeaker();

    // Configure the trigger bindings
    configureBindings();

    Command driveAngularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driver.getRightX());
    
    Command drivePresetAdvanced = drivebase.drivePresetAdvancedCommand(
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> driver.getRightX(),
        () -> driver.getRightY(),
              driver.getYButtonPressed(), driver.getAButtonPressed(), 
              driver.getXButtonPressed(), driver.getBButtonPressed());

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? drivePresetAdvanced : drivePresetAdvanced);
  }

  private void configureBindings() {
    driver.Start.onTrue(Commands.runOnce(drivebase::zeroGyro));
    driver.PovUp.onTrue(drivebase.setModifierCommand(0.5))
                .onFalse(drivebase.setModifierCommand(1.0));
    driver.PovRight.onTrue(drivebase.changeDriveMode(DriveMode.NOTE));
    driver.PovDown.onTrue(drivebase.changeDriveMode(DriveMode.ROBOT_RELATIVE));
    driver.PovLeft.onTrue(drivebase.changeDriveMode(DriveMode.ABSOLUTE));
    driver.PovUpRight.onTrue(drivebase.changeDriveMode(DriveMode.TELEOP));
    driver.LB.onTrue(drivebase.changeDriveMode(DriveMode.SPEAKER)); 
    driver.RB.onTrue(drivebase.changeDriveMode(DriveMode.AMP));

    // Auto Align Shooter + Rotate to Speaker
    operator.X.whileTrue(new PivotCommand(m_pivotSubsystem, () -> m_pivotSubsystem.autoAlignShooter()));
    operator.X.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.Shooter.shooterRPS));
    operator.X.onTrue(m_pivotSubsystem.setShooterAutonCommand());
    operator.X.onTrue(drivebase.changeDriveMode(DriveMode.SPEAKER));

    // Climb
    operator.RB.whileTrue(new ClimbCommand(m_climbSubsystem, () -> 1.0));
    operator.LB.whileTrue(new ClimbCommand(m_climbSubsystem, () -> -1.0));

    // Shoot In Amp 
    operator.B.onTrue(m_pivotSubsystem.setShooterAutonCommand());
    operator.B.onTrue(m_shooterSubsystem.changeAmpTopMultiplierCommand());
    operator.B.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.GamePieces.amp.speedToShoot));
    operator.B.whileTrue(new PivotCommand(m_pivotSubsystem, () -> Constants.GamePieces.amp.angleToshoot));

    // Shoot At Subwoofer
    operator.A.whileTrue(new PivotCommand(m_pivotSubsystem, () -> Constants.GamePieces.speaker.angle));
    operator.A.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.Shooter.shooterRPS));

    // operator.Y.whileTrue(new PivotCommand(m_pivotSubsystem, () -> m_pivotSubsystem.autoAlignShooter()));
    // operator.Y.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> 20.0));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  public double getIndexControl() {
    double leftTrigger = DoubleUtils.normalize(-operator.getLeftTrigger());
    leftTrigger = JoystickHelper.SimpleAxialDeadzone(leftTrigger, Constants.Input.kTriggerDeadband);

    double leftJoystick = DoubleUtils.normalize(-operator.getLeftY());
    leftJoystick = JoystickHelper.SimpleAxialDeadzone(leftJoystick, Constants.Input.kJoystickDeadband);

    if (m_indexSystem.getBeambreak() == false) {
      return leftTrigger;
    } else {
      return leftJoystick * 0.2;
    }
  }

  public double getIntakeControl() {
    double leftJoystick = DoubleUtils.normalize(-operator.getLeftY());
    leftJoystick = JoystickHelper.SimpleAxialDeadzone(leftJoystick, Constants.Input.kJoystickDeadband);

    if (m_indexSystem.getBeambreak() == true) {
      return leftJoystick;
    }
    return 0.0;
  }

  public double getPivotControl() {
    if (m_pivotSubsystem.getShooterAutonTriggered() == false) {
      double newRightJoystickValue = DoubleUtils.normalize(-operator.getRightY());
      newRightJoystickValue = JoystickHelper.SimpleAxialDeadzone(newRightJoystickValue, Constants.Input.kJoystickDeadband);
      double newShooterDegree = m_pivotSubsystem.currentShooterDegree + newRightJoystickValue;
      m_pivotSubsystem.currentShooterDegree = DoubleUtils.clamp(newShooterDegree, Constants.Shooter.shooterStartDegree,
          Constants.Shooter.shooterEndDegree);
    }
    return m_pivotSubsystem.currentShooterDegree;
  }

  public double getShooterControl() {
    double right_trigger = DoubleUtils.normalize(operator.getRightTrigger());
    right_trigger = JoystickHelper.SimpleAxialDeadzone(right_trigger, Constants.Input.kTriggerDeadband);

    double control = right_trigger * Constants.Shooter.shooterRPS; // converting into RPS
    return control;
  }

  public void zeroGyroOnTeleop() {
    drivebase.zeroGyro();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void stopRumble() {
    driver.setLeftRumble(0.0);
    driver.setRightRumble(0.0);
    operator.setLeftRumble(0.0);
    operator.setRightRumble(0.0);
  }

  public void disabledActions() {
    drivebase.setSpeaker();
  }

  // Gets rid of the yellow errors in Robot.java
  public void ewyellowerrors() {
  }
}
