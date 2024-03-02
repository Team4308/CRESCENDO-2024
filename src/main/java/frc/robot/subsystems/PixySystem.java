package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pixy2api.Pixy2;
import frc.robot.pixy2api.Pixy2CCC;
import frc.robot.pixy2api.Pixy2CCC.Block;
import frc.robot.pixy2api.links.SPILink;
import edu.wpi.first.util.sendable.Sendable;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class PixySystem extends LogSubsystem {
  private Pixy2 pixy;
  private int sig = Pixy2CCC.CCC_SIG1;

  public PixySystem() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();
    pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
		pixy.setLED(255, 255, 255); // Sets the RGB LED to full white
  }

  @Override
  public void periodic() {
    //getClosestTarget();
    //getTargetWidth(getClosestTarget());
    //getTargetX(getClosestTarget());
    // This method will be called once per scheduler run
  }

  public Block getClosestTarget() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 25, for a slight increase in efficiency
		int targetCount = pixy.getCCC().getBlocks(false, sig, 10);
		if (targetCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
    SmartDashboard.putNumber("Targets", targetCount);
		ArrayList<Block> targets = pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
		Block closestTarget = null;
		for (Block target : targets) { // Loops through all blocks and finds the widest one
			if (closestTarget == null) {
				closestTarget = target;
			} else if (target.getWidth() > closestTarget.getWidth()) {
				closestTarget = target;
			}
		}

		return closestTarget;
	}

  public double getTargetWidth(Block target) {
    if (target == null) {
      return 0; 
    }
    int targetWidth = target.getWidth();
    SmartDashboard.putNumber("Target Width", targetWidth);
    return targetWidth;
  }

  public double getTargetX(Block target) {
    if (target == null) {
      return 0; // do nothing if target not in view (not sure if this will work)
    }
    int targetX = target.getX();
    targetX -= 157.5; // range from -157.5 to 157.5
    SmartDashboard.putNumber("Target X", targetX);
    return targetX;
  }

  public double getTargetY(Block target) {
    if (target == null) {
      return 0; // do nothing if target not in view (not sure if this will work)
    }
    int targetY = target.getY();
    targetY -= 103.5; // range from -103.5 to 103.5
    SmartDashboard.putNumber("Target Y", targetY);
    return targetY;
  }

  @Override
    public Sendable log() {
        return this;
    }

}