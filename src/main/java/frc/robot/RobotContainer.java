package frc.robot;

import java.io.File;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.XBoxWrapper;
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
import frc.robot.commands.LEDCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.BeambreakCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.PixySystem;
import frc.robot.subsystems.RotateShooterSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexSystem;

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
  
  // Subsystems
  private final IntakeSystem m_intakeSystem;
  private final LEDSystem m_ledSystem;
  private final RotateShooterSystem m_rotateShooterSystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ClimbSubsystem m_climbSubsystem;
  private final IndexSystem m_indexSystem;
  private final PixySystem m_pixySystem;

  // Commands
  private final IntakeCommand intakeCommand;
  private final LEDCommand ledCommand;
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

  // LED
  private Integer debounce = 0;
  private Double prev = 0.0;
  
  private DigitalInput shooterBeambrake;
  public double shooterDegree = 45;
  
  // State Machines
  private boolean shooterAutonTriggered = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    setSpeaker();

    //Subsystem Instantiations
    m_intakeSystem = new IntakeSystem();
    subsystems.add(m_intakeSystem);

    m_ledSystem = new LEDSystem();
    subsystems.add(m_ledSystem);
    
    m_rotateShooterSystem = new RotateShooterSystem();
    subsystems.add(m_rotateShooterSystem);
    
    m_shooterSubsystem = new ShooterSubsystem();
    subsystems.add(m_shooterSubsystem);
    
    m_climbSubsystem = new ClimbSubsystem();
    subsystems.add(m_climbSubsystem);
    
    m_indexSystem = new IndexSystem();
    subsystems.add(m_indexSystem);

    m_pixySystem = new PixySystem();
    subsystems.add(m_pixySystem);

    NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_intakeSystem, () -> -1.0));
    NamedCommands.registerCommand("IndexCommand", new IndexCommand(m_indexSystem, () -> -1.0));
    NamedCommands.registerCommand("ShooterCommand", new ShooterCommand(m_shooterSubsystem, () -> 20.0));
    NamedCommands.registerCommand("SpeakerAlignTrue", new InstantCommand(() -> drivebase.alignToSpeaker(true)));
    NamedCommands.registerCommand("SpeakerAlignFalse", new InstantCommand(() -> drivebase.alignToSpeaker(false)));
    NamedCommands.registerCommand("ResetGyro", new InstantCommand(drivebase::zeroGyro));
    NamedCommands.registerCommand("AutoAlignShooter", new RotateShooterCommand(m_rotateShooterSystem, () -> m_rotateShooterSystem.autoAlignShooter()));
    NamedCommands.registerCommand("BeambreakCommand", new BeambreakCommand(() -> getBeambreakControl()));
    
    //Command Instantiations
    intakeCommand = new IntakeCommand(m_intakeSystem, () -> getIntakeControl());
    m_intakeSystem.setDefaultCommand(intakeCommand);
    
    ledCommand = new LEDCommand(m_ledSystem, () -> getLEDCommand());
    m_ledSystem.setDefaultCommand(ledCommand);
    
    rotateShooterCommand = new RotateShooterCommand(m_rotateShooterSystem, () -> getRotateShooterControl());
    m_rotateShooterSystem.setDefaultCommand(rotateShooterCommand);
    
    ShooterCommand = new ShooterCommand(m_shooterSubsystem, () -> getShooterControl());
    m_shooterSubsystem.setDefaultCommand(ShooterCommand);
    
    climbCommand = new ClimbCommand(m_climbSubsystem, () -> 0.0);
    m_climbSubsystem.setDefaultCommand(climbCommand);

    indexCommand = new IndexCommand(m_indexSystem, () -> indexCommand());
    m_indexSystem.setDefaultCommand(indexCommand);
    
    autonomousChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autonomousChooser);

    shooterBeambrake = new DigitalInput(Constants.Mapping.Shooter.beambrake);
    
    // Configure the trigger bindings
    configureBindings();
    
    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
    //                                                                                             OperatorConstants.LEFT_X_DEADBAND),
    //                                                                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
    //                                                                                             OperatorConstants.RIGHT_X_DEADBAND),
    //                                                                driverXbox.getHID()::getYButtonPressed,
    //                                                                driverXbox.getHID()::getAButtonPressed,
    //                                                                driverXbox.getHID()::getXButtonPressed,
    //                                                                driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> stick.getRightX(),
    //     () -> stick.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -stick.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> stick.getLeftTrigger());

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
 
    stick.Y.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    stick.LB.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    stick.A.onTrue(new InstantCommand(() -> drivebase.alignToSpeaker(true)));
    stick.A.onFalse(new InstantCommand(() -> drivebase.alignToSpeaker(false)));
    stick.X.onTrue(new InstantCommand(() -> drivebase.alignToNote(true)));
    stick.X.onFalse(new InstantCommand(() -> drivebase.alignToNote(false)));
    stick.B.whileTrue(Commands.deferredProxy(() -> drivebase.alignToAmp()));
    stick.B.onTrue(new InstantCommand(() -> setAmp()));
    stick.B.onFalse(new InstantCommand(() -> setSpeaker()));
    stick.Back.onTrue(new InstantCommand(() -> m_rotateShooterSystem.changeGoalHeight(1.0)));
    stick.Start.onTrue(new InstantCommand(() -> m_rotateShooterSystem.changeGoalHeight(-1.0)));
    stick.RB.onTrue(new InstantCommand(() -> drivebase.fieldRelative(false)));
    stick.RB.onFalse(new InstantCommand(() -> drivebase.fieldRelative(true)));
    stick1.Y.whileTrue(new LEDCommand(m_ledSystem, () -> 0.69)); // yellow

    //auto align shooter
    stick1.X.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> m_rotateShooterSystem.autoAlignShooter()));
    stick1.X.onTrue(new InstantCommand(() -> setShooterAutonTriggered(true)));
    stick1.X.onFalse(new InstantCommand(() -> setShooterAutonTriggered(false)));

    //climb
    stick1.RB.whileTrue(new ClimbCommand(m_climbSubsystem, () -> 0.5));
    stick1.LB.whileTrue(new ClimbCommand(m_climbSubsystem, () -> -0.5));
    stick1.RB.onFalse(new InstantCommand(() -> m_climbSubsystem.stopControllers()));
    stick1.LB.onFalse(new InstantCommand(() -> m_climbSubsystem.stopControllers()));

    stick1.A.onTrue(new InstantCommand(() -> m_rotateShooterSystem.resetSensors()));  // debugging

    //shoot in amp
    stick1.B.onTrue(new InstantCommand(() -> setShooterAutonTriggered(true)));
    stick1.B.onTrue(new InstantCommand(() -> m_shooterSubsystem.changeTopMultiplier(Constants.Shooter.shootInAmpMultiplier)));
    stick1.B.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.GamePieces.amp.speedToShoot));
    stick1.B.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> Constants.GamePieces.amp.angleToshoot));
    stick1.B.onFalse(new InstantCommand(() -> setShooterAutonTriggered(false)));
    stick1.B.onFalse(new InstantCommand(() -> m_shooterSubsystem.changeTopMultiplier(1)));
  }

  public Command getAutonomousCommand()
  {
    return autonomousChooser.getSelected();
  }
    
  public Command setShooterAutonTriggered(boolean value) {
    shooterAutonTriggered = value;
    return null;
  }

  public double indexCommand() {
    double trig = stick1.getLeftTrigger()*-1;
    if (-0.06 <= trig && trig <= 0.06) {
      trig = 0;
    }
    double joy = stick1.getLeftY()*-1;
    if (-0.06 <= joy && joy <= 0.06) {
      joy = 0.0;
    }
    if (shooterBeambrake.get() == false) {
      return trig;
    } else {
      return joy * 0.25;
    }
  }

  public Double getLEDCommand() {
    if(RobotController.isBrownedOut()) {
      prev = 0.67;
      return 0.67; // red-orange
    }
    if (getShooterControl() != 0.0){
      prev = getShooterControl();
      return getShooterControl(); // trigger colourOutputShooter
    }
    if(PixySystem.getClosestTarget() != null) {
      // target in range
      debounce++;
      if(debounce == 5) debounce = 2;
      prev = -0.09; // strobe blue
      return -0.09;
    }
    if(PixySystem.getClosestTarget() == null) {
      // no target
      debounce--;
      if(debounce <= 0) debounce = 0;
    }
    if(debounce == 0) {
      prev = -0.39;
      return -0.39;
    }  // default enabled, colour waves lava

    return prev;
    // disabled state is slow rgb
  }

  public double getIntakeControl() {
    if (shooterBeambrake.get() == true) {
      return stick1.getLeftY()*-1;
    }
    return 0.0;
  }
    
  public double getRotateShooterControl(){
    if (shooterAutonTriggered == false) {
      var newVal = stick1.getRightY();
      if (-0.06 <= newVal && newVal <= 0.06) {  //deadband; too lazy to code properly
        newVal = 0;
      }
      double newShooterDegree = shooterDegree + newVal;
      if (Constants.Shooter.shooterStartDegree <= newShooterDegree && newShooterDegree <= Constants.Shooter.shooterEndDegree) {//could use more fine tuning
        shooterDegree = newShooterDegree;
      }
    }
    return shooterDegree; 
  }
  
  public double getShooterControl() {
    return stick1.getRightTrigger() * 100;  //converting into RPS
  }

  public boolean getBeambreakControl() {
    return shooterBeambrake.get();
  }

  public void zeroGyroOnTeleop()
  {
    drivebase.zeroGyro();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake); 
  }

  public void stopRumble() {
    stick.setLeftRumble(0.0);
    stick.setRightRumble(0.0);
    stick1.setLeftRumble(0.0);
    stick1.setRightRumble(0.0);
  }

  public void setSpeaker() {
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
  
  // Gets rid of the yellow errors in Robot.java
  public void ewyellowerrors() {}
}
