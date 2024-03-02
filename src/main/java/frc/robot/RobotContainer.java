package frc.robot;

import frc.robot.commands.RotateShooterCommand;
import frc.robot.subsystems.RotateShooterSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import ca.team4308.absolutelib.control.XBoxWrapper;


public class RobotContainer {
  // Subsystems
  private final RotateShooterSystem m_rotateShooterSystem;

  // Commands
  private final RotateShooterCommand rotateShooterCommand;

  // Controllers
  public final XBoxWrapper stick = new XBoxWrapper(0);

  public double shooterDegree = 20.0;

  public RobotContainer() {
    m_rotateShooterSystem = new RotateShooterSystem();

    rotateShooterCommand = new RotateShooterCommand(m_rotateShooterSystem, () -> getRotateShooterControl());

    m_rotateShooterSystem.setDefaultCommand(rotateShooterCommand);

    // Configure the trigger bindings
    configureBindings();
  }
  
  private void configureBindings() {
    stick.A.onTrue(new InstantCommand(() -> m_rotateShooterSystem.resetSensors()));//debugging
  }

  public double getRotateShooterControl(){
    double testVale = shooterDegree + stick.getRightY();
    if (15 <= testVale && testVale <= 43) {//could use more fine tuning
      shooterDegree = testVale;
    }
    return shooterDegree;
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
