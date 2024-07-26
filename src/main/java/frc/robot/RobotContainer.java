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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.BeambreakCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.RotateShooterSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexSystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  Double modifier = 1.0;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;

  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  // Subsystems
  private final IntakeSystem m_intakeSystem;
  private final RotateShooterSystem m_rotateShooterSystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ClimbSubsystem m_climbSubsystem;
  private final IndexSystem m_indexSystem;
  // private final PixySystem m_pixySystem;

  // Commands
  private final IntakeCommand intakeCommand;
  private final RotateShooterCommand rotateShooterCommand;
  private final ShooterCommand ShooterCommand;
  private final ClimbCommand climbCommand;
  private final IndexCommand indexCommand;

  // Controllers
  // For swerve
  // final CommandXboxController driverXbox = new CommandXboxController(0);
  public final XBoxWrapper stick = new XBoxWrapper(Constants.Mapping.Controllers.kStick);
  public final XBoxWrapper stick1 = new XBoxWrapper(Constants.Mapping.Controllers.kStick1);

  // Auto
  private final SendableChooser<Command> autonomousChooser;

  private DigitalInput shooterBeambrake;
  public double currentShooterDegree = 18;

  // State Machines
  private boolean shooterAutonTriggered = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setSpeaker();

    // Subsystem Instantiations
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
        "swerve"));
    subsystems.add(drivebase);

    m_intakeSystem = new IntakeSystem();
    subsystems.add(m_intakeSystem);

    // m_ledSystem = new LEDSystem();
    // subsystems.add(m_ledSystem);

    m_rotateShooterSystem = new RotateShooterSystem();
    subsystems.add(m_rotateShooterSystem);

    m_shooterSubsystem = new ShooterSubsystem();
    subsystems.add(m_shooterSubsystem);

    m_climbSubsystem = new ClimbSubsystem();
    subsystems.add(m_climbSubsystem);

    m_indexSystem = new IndexSystem();
    subsystems.add(m_indexSystem);

    // m_pixySystem = new PixySystem();
    // subsystems.add(m_pixySystem);

    NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_intakeSystem, () -> -1.0));
    NamedCommands.registerCommand("IndexCommand", new IndexCommand(m_indexSystem, () -> -0.15));
    NamedCommands.registerCommand("IndexShootCommand", new IndexCommand(m_indexSystem, () -> -1.0));
    NamedCommands.registerCommand("ShooterCommand",
        new ShooterCommand(m_shooterSubsystem, () -> Constants.Shooter.shooterRPS));
    NamedCommands.registerCommand("ShooterStop", new ShooterCommand(m_shooterSubsystem, () -> 0.0));
    NamedCommands.registerCommand("ShooterDisruption", new ShooterCommand(m_shooterSubsystem, () -> 25.0));
    // NamedCommands.registerCommand("SpeakerAlignTrue", new InstantCommand(() ->
    // drivebase.alignToSpeaker(true)));
    // NamedCommands.registerCommand("SpeakerAlignFalse", new InstantCommand(() ->
    // drivebase.alignToSpeaker(false)));
    NamedCommands.registerCommand("SpeakerAlign", new InstantCommand(() -> drivebase.speakerAlignCommand()));
    NamedCommands.registerCommand("ResetGyro", new InstantCommand(drivebase::zeroGyro));
    NamedCommands.registerCommand("AutoAlignShooter",
        new RotateShooterCommand(m_rotateShooterSystem, () -> m_rotateShooterSystem.autoAlignShooter()));
    NamedCommands.registerCommand("BeambreakCommand", new BeambreakCommand(() -> getBeambreakControl()));
    NamedCommands.registerCommand("SubwooferAngle",
        new RotateShooterCommand(m_rotateShooterSystem, () -> Constants.GamePieces.speaker.angle));
    NamedCommands.registerCommand("SubwooferSideAngle", new RotateShooterCommand(m_rotateShooterSystem, () -> 60.0));
    NamedCommands.registerCommand("AmpAngle",
        new RotateShooterCommand(m_rotateShooterSystem, () -> Constants.GamePieces.amp.angleToshoot));
    // Command Instantiations
    intakeCommand = new IntakeCommand(m_intakeSystem, () -> getIntakeControl());
    m_intakeSystem.setDefaultCommand(intakeCommand);

    // ledCommand = new LEDCommand(m_ledSystem, () -> getLEDCommand());
    // m_ledSystem.setDefaultCommand(ledCommand);

    rotateShooterCommand = new RotateShooterCommand(m_rotateShooterSystem, () -> getRotateShooterControl());
    m_rotateShooterSystem.setDefaultCommand(rotateShooterCommand);

    ShooterCommand = new ShooterCommand(m_shooterSubsystem, () -> getShooterControl());
    m_shooterSubsystem.setDefaultCommand(ShooterCommand);

    climbCommand = new ClimbCommand(m_climbSubsystem, () -> 0.0);
    m_climbSubsystem.setDefaultCommand(climbCommand);

    indexCommand = new IndexCommand(m_indexSystem, () -> getIndexControl());
    m_indexSystem.setDefaultCommand(indexCommand);

    autonomousChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autonomousChooser);

    shooterBeambrake = new DigitalInput(Constants.Mapping.Shooter.beambrake);

    // Configure the trigger bindings
    configureBindings();

    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    // () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
    // OperatorConstants.LEFT_Y_DEADBAND),
    // () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
    // OperatorConstants.LEFT_X_DEADBAND),
    // () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
    // OperatorConstants.RIGHT_X_DEADBAND),
    // driverXbox.getHID()::getYButtonPressed,
    // driverXbox.getHID()::getAButtonPressed,
    // driverXbox.getHID()::getXButtonPressed,
    // driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    // () -> MathUtil.applyDeadband(stick.getLeftY(),
    // OperatorConstants.LEFT_Y_DEADBAND),
    // () -> MathUtil.applyDeadband(stick.getLeftX(),
    // OperatorConstants.LEFT_X_DEADBAND),
    // () -> stick.getRightX(),
    // () -> stick.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * modifier,
        () -> -MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * modifier,
        () -> -stick.getRightX() * modifier);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> stick.getLeftTrigger());

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    stick.Y.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    stick.A.onTrue(new InstantCommand(() -> drivebase.alignToSpeaker(true)));
    stick.A.onFalse(new InstantCommand(() -> drivebase.alignToSpeaker(false)));
    stick.X.onTrue(new InstantCommand(() -> drivebase.alignToNote(true)));
    stick.X.onFalse(new InstantCommand(() -> drivebase.alignToNote(false)));
    stick.B.onTrue(new InstantCommand(() -> setAmp()));
    stick.B.onTrue(new InstantCommand(() -> drivebase.alignToAmp(true)));
    stick.B.onFalse(new InstantCommand(() -> drivebase.alignToAmp(false)));
    stick.B.onFalse(new InstantCommand(() -> setSpeaker()));
    stick.RB.onTrue(new InstantCommand(() -> drivebase.fieldRelative(false)));
    stick.RB.onFalse(new InstantCommand(() -> drivebase.fieldRelative(true)));
    stick.LB.onTrue(new InstantCommand(() -> setModifer(0.2)));
    stick.LB.onFalse(new InstantCommand(() -> setModifer(1.0)));
    // stick1.Y.whileTrue(new LEDCommand(m_ledSystem, () -> 0.69)); // yellow

    // auto align shooter
    stick1.X.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> m_rotateShooterSystem.autoAlignShooter()));
    stick1.X.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.Shooter.shooterRPS));
    stick1.X.onTrue(new InstantCommand(() -> setShooterAutonTriggered(true)));
    stick1.X.onFalse(new InstantCommand(() -> setShooterAutonTriggered(false)));

    // climb
    stick1.RB.whileTrue(new ClimbCommand(m_climbSubsystem, () -> 1.0));
    stick1.LB.whileTrue(new ClimbCommand(m_climbSubsystem, () -> -1.0));
    stick1.RB.onFalse(new InstantCommand(() -> m_climbSubsystem.stopControllers()));
    stick1.LB.onFalse(new InstantCommand(() -> m_climbSubsystem.stopControllers()));

    // shoot in amp
    stick1.B.onTrue(new InstantCommand(() -> setShooterAutonTriggered(true)));
    stick1.B.onTrue(
        new InstantCommand(() -> m_shooterSubsystem.changeTopMultiplier(Constants.Shooter.shootInAmpMultiplier)));
    stick1.B.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.GamePieces.amp.speedToShoot));
    stick1.B.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> Constants.GamePieces.amp.angleToshoot));
    stick1.B.onFalse(new InstantCommand(() -> setShooterAutonTriggered(false)));
    stick1.B.onFalse(new InstantCommand(() -> m_shooterSubsystem.changeTopMultiplier(1)));

    stick1.A.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> Constants.GamePieces.speaker.angle));
    stick1.A.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.Shooter.shooterRPS));

    stick1.Y.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> m_rotateShooterSystem.autoAlignShooter()));
    stick1.Y.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> 20.0));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  public Command setShooterAutonTriggered(boolean value) {
    shooterAutonTriggered = value;
    return null;
  }

  public double getIndexControl() {
    double leftTrigger = DoubleUtils.normalize(-stick1.getLeftTrigger());
    leftTrigger = JoystickHelper.SimpleAxialDeadzone(leftTrigger, 0.06);

    double leftJoystick = DoubleUtils.normalize(-stick1.getLeftY());
    leftJoystick = JoystickHelper.SimpleAxialDeadzone(leftTrigger, 0.06);

    if (shooterBeambrake.get() == false) {
      return leftTrigger;
    } else {
      return leftJoystick * 0.2;
    }
  }

  public double getIntakeControl() {
    double leftJoystick = DoubleUtils.normalize(-stick1.getLeftY());
    leftJoystick = JoystickHelper.SimpleAxialDeadzone(leftJoystick, 0.06);

    if (shooterBeambrake.get() == true) {
      return leftJoystick;
    }
    return 0.0;
  }

  public double getRotateShooterControl() {
    if (shooterAutonTriggered == false) {
      double newRightJoystickValue = DoubleUtils.normalize(-stick1.getRightY());
      newRightJoystickValue = JoystickHelper.SimpleAxialDeadzone(newRightJoystickValue, 0.06);

      double newShooterDegree = currentShooterDegree + newRightJoystickValue;
      currentShooterDegree = DoubleUtils.clamp(newShooterDegree, Constants.Shooter.shooterStartDegree,
          Constants.Shooter.shooterEndDegree);
    }
    return currentShooterDegree;
  }

  public double getShooterControl() {
    double right_trigger = DoubleUtils.normalize(stick1.getRightTrigger());
    right_trigger = JoystickHelper.SimpleAxialDeadzone(right_trigger, 0.06);

    double control = right_trigger * Constants.Shooter.shooterRPS; // converting into RPS
    return control;
  }

  public boolean getBeambreakControl() {
    return shooterBeambrake.get();
  }

  public void zeroGyroOnTeleop() {
    drivebase.zeroGyro();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void stopRumble() {
    stick.setLeftRumble(0.0);
    stick.setRightRumble(0.0);
    stick1.setLeftRumble(0.0);
    stick1.setRightRumble(0.0);
  }

  public void setSpeaker() {
    if (DriverStation.getAlliance().isEmpty())
      return;

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      LimelightHelpers.setPipelineIndex("", 0);
    } else {
      LimelightHelpers.setPipelineIndex("", 2);
    }
  }

  public void setAmp() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      LimelightHelpers.setPipelineIndex("", 1);
    } else {
      LimelightHelpers.setPipelineIndex("", 3);
    }
  }

  public void setModifer(Double value) {
    modifier = value;
  }

  public void disabledActions() {
    setSpeaker();
  }

  // Gets rid of the yellow errors in Robot.java
  public void ewyellowerrors() {
  }
}
