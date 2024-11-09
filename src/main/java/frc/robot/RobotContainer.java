package frc.robot;

import java.io.File;
import java.util.ArrayList;

import com.ctre.phoenix.schedulers.SequentialScheduler;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Controller;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem.DriveMode;

public class RobotContainer {

  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  // Subsystems
  private final SwerveSubsystem drivebase;

  // Commands

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

    configureNamedCommands();
    // fix these commands later ***
    
    // Command Instantiations

    autonomousChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autonomousChooser);

    // Configure the trigger bindings
    configureBindings();

    Command driveAngularVelocity = drivebase.driveVelocityCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), Controller.Driver.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), Controller.Driver.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getRightX(), Controller.Driver.RIGHT_X_DEADBAND));
    
    Command drivePresetAdvanced = drivebase.drivePresetsCommand(
        () -> MathUtil.applyDeadband(driver.getLeftX(), Controller.Driver.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftY(), Controller.Driver.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getRightX(), Controller.Driver.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getRightY(), Controller.Driver.RIGHT_Y_DEADBAND),
              driver.getYButtonPressed(), driver.getAButtonPressed(), 
              driver.getXButtonPressed(), driver.getBButtonPressed());

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveAngularVelocity : drivePresetAdvanced);
  }

  private void configureBindings() {
    driver.Up.onTrue(Commands.runOnce(drivebase::zeroGyro));
    driver.LeftTrigger.onTrue(drivebase.changeDriveMode(DriveMode.ROBOT_RELATIVE))
                      .onFalse(drivebase.changeDriveMode(DriveMode.VELOCITY_ADV));

    driver.RB.onTrue(drivebase.changeDriveMode(DriveMode.SPEAKER))
             .onFalse(drivebase.changeDriveMode(DriveMode.VELOCITY_ADV));

    driver.LB.onTrue(drivebase.changeDriveMode(DriveMode.AMP))
             .onFalse(drivebase.changeDriveMode(DriveMode.VELOCITY_ADV));

    driver.Down.onTrue(Commands.runOnce(drivebase::setupPathPlanner));
    // TESTING ONLY FOR TUNING PATHPLANNER PIDS, DISABLE WHEN COMPETING
  }

  public void configureNamedCommands() {
  } 

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
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
  }

  // Gets rid of the yellow errors in Robot.java
  public void ewyellowerrors() {
  }
}
