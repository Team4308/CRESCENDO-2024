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

  //state machines
  private boolean shooterAutonTriggered = false;

  public RobotContainer() {
    m_rotateShooterSystem = new RotateShooterSystem();

    rotateShooterCommand = new RotateShooterCommand(m_rotateShooterSystem, getRotateShooterControl());

    m_rotateShooterSystem.setDefaultCommand(rotateShooterCommand);

    // Configure the trigger bindings
    configureBindings();
  }
  
  private void configureBindings() {
    stick.A.onTrue(new InstantCommand(() -> m_rotateShooterSystem.resetSensors()));//debugging
    stick.B.whileTrue(new InstantCommand(() -> m_rotateShooterSystem.autoAlignShooter()));
    stick.B.onTrue(new InstantCommand(() -> setShooterAutonTriggered(true)));
    stick.B.onFalse(new InstantCommand(() -> setShooterAutonTriggered(false)));
  }

  public double getRotateShooterControl(){
    if (shooterAutonTriggered == false) {
      double newShooterDegree = shooterDegree + stick.getRightY();
      if (16 <= newShooterDegree && newShooterDegree <= 43) {//could use more fine tuning
        shooterDegree = newShooterDegree;
      }
    }
    
    return shooterDegree;
  }

  public Command setShooterAutonTriggered(boolean value) {
    shooterAutonTriggered = value;
    return null;
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
