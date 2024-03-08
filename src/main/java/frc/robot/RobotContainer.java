package frc.robot;

import java.io.File;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.pathplanner.lib.auto.NamedCommands;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDCommand;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.ShooterCommand;
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
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  // LED
  private Integer debounce = 0;
  private Double prev = 0.0;
  
  private DigitalInput shooterBeambrake;
  public double shooterDegree = 20.0;
  
  // State Machines
  private boolean shooterAutonTriggered = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
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

    NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_intakeSystem, () -> 1.0));
    NamedCommands.registerCommand("IndexCommand", new InstantCommand(() -> m_indexSystem.setIndexOutput(-1.0)));
    NamedCommands.registerCommand("ShooterCommand", new ShooterCommand(m_shooterSubsystem, () -> 1200.0));
    NamedCommands.registerCommand("AlignToSpeaker", new InstantCommand(drivebase::alignToSpeaker));
    NamedCommands.registerCommand("AutoAlignShooter", new InstantCommand(() -> m_rotateShooterSystem.autoAlignShooter()));
    NamedCommands.registerCommand("SetShooterAlignTrue", new InstantCommand(() -> setShooterAutonTriggered(true))); // not needed?
    NamedCommands.registerCommand("SetShooterAlignFalse", new InstantCommand(() -> setShooterAutonTriggered(false))); // not needed?
    
    //Command Instantiations
    intakeCommand = new IntakeCommand(m_intakeSystem, () -> getIntakeControl());
    m_intakeSystem.setDefaultCommand(intakeCommand);
    
    ledCommand = new LEDCommand(m_ledSystem, () -> getLEDCommand());
    m_ledSystem.setDefaultCommand(ledCommand);
    
    rotateShooterCommand = new RotateShooterCommand(m_rotateShooterSystem, getRotateShooterControl());
    m_rotateShooterSystem.setDefaultCommand(rotateShooterCommand);
    
    ShooterCommand = new ShooterCommand(m_shooterSubsystem, () -> getShooterControl());
    m_shooterSubsystem.setDefaultCommand(ShooterCommand);
    
    climbCommand = new ClimbCommand(m_climbSubsystem, () -> climbControl());
    m_climbSubsystem.setDefaultCommand(climbCommand);

    indexCommand = new IndexCommand(m_intakeSystem, () -> indexCommand());
    m_indexSystem.setDefaultCommand(indexCommand);

    SmartDashboard.putData(autoCommandChooser);

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
        () -> MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> stick.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> stick.getLeftTrigger());

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
 
    stick.A.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    stick.B.whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              )); 
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    stick.Y.onTrue(new InstantCommand(drivebase::alignToSpeaker));
    // stick.LB.onTrue(new InstantCommand(() -> drivebase.alignToNote()));
    stick.X.onTrue(new InstantCommand(() -> m_ledSystem.setOutput(0.69))); // yellow
    // stick.Start.onTrue(new InstantCommand(() -> m_rotateShooterSystem.resetSensors()));//debugging
    stick1.X.whileTrue(new InstantCommand(() -> m_rotateShooterSystem.autoAlignShooter()));
    stick1.X.onTrue(new InstantCommand(() -> setShooterAutonTriggered(true)));
    stick1.X.onFalse(new InstantCommand(() -> setShooterAutonTriggered(false)));
    stick1.RB.whileTrue(new InstantCommand(() -> m_climbSubsystem.setMotorOutput(TalonSRXControlMode.PercentOutput, 1)));
    stick1.LB.whileTrue(new InstantCommand(() -> m_climbSubsystem.setMotorOutput(TalonSRXControlMode.PercentOutput, -1)));
    stick1.RB.onFalse(new InstantCommand(() -> m_climbSubsystem.stopControllers()));
    stick1.LB.onFalse(new InstantCommand(() -> m_climbSubsystem.stopControllers()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("TestAuto");
  }
    
  public Command setShooterAutonTriggered(boolean value) {
    shooterAutonTriggered = value;
    return null;
  }

  public double indexCommand() {
    double trig = stick1.getRightTrigger()*-1;
    if (-0.06 <= trig && trig <= 0.06) {
      trig = 0;
    }
    double joy = stick1.getLeftY()*-1;
    if (-0.06 <= joy && joy <= 0.06) {
      joy = 0;
    }
    if (shooterBeambrake.get() == true) {
      return trig;
    } else {
      return joy;
    }
  }

  public Double getLEDCommand() {
    
    if(RobotController.isBrownedOut()) {
      prev = 0.67;
      return 0.67; // red-orange
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
    if (shooterBeambrake.get() == false) {
      return stick1.getLeftY()*-1;
    }
    return 0.0;
  }
    
  public double getRotateShooterControl() {
    if (shooterAutonTriggered == false) {
      double newShooterDegree = shooterDegree + stick.getRightY();
      if (Constants.Shooter.shooterStartDegree <= newShooterDegree && newShooterDegree <= Constants.Shooter.shooterEndDegree) {//could use more fine tuning
        shooterDegree = newShooterDegree;
      }
    } 
    return shooterDegree;
  }
  
  public double getShooterControl() {
    return stick.getRightTrigger() * 1200;//converting into RPM
  }
    
  public double climbControl(){
    return stick.getLeftTrigger();
  }

  public void setDriveMode()
  {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake); 
  }
  
  // Gets rid of the yellow errors in Robot.java
  public void ewyellowerrors() {}
}