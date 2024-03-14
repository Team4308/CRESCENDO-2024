package frc.robot.subsystems;

import java.util.ArrayList;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.pixy2api.Pixy2;
import frc.robot.pixy2api.Pixy2CCC;
import frc.robot.pixy2api.Pixy2CCC.Block;
import frc.robot.pixy2api.links.SPILink;


public class PixySystem extends LogSubsystem {
  public static Pixy2 pixy;
  private static int sig = Pixy2CCC.CCC_SIG1;

  public PixySystem() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();
    pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
		pixy.setLED(255, 255, 255); // Sets the RGB LED to full white
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Block getClosestTarget() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 25, for a slight increase in efficiency
		int targetCount = pixy.getCCC().getBlocks(false, sig, 10);
		if (targetCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
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

  public static int getTargetWidth(Block target) {
    if (target == null) {
      return 0; 
    }
    int targetWidth = target.getWidth();
    return targetWidth;
  }

  public static int getTargetX(Block target) {
    if (target == null) {
      return 0; // do nothing if target not in view (not sure if this will work)
    }
    int targetX = target.getX();
    targetX -= 157.5; // range from -157.5 to 157.5
    return targetX;
  }

  public static int getTargetY(Block target) {
    if (target == null) {
      return 0; // do nothing if target not in view (not sure if this will work)
    }
    int targetY = target.getY();
    targetY -= 103.5; // range from -103.5 to 103.5
    return targetY;
  }

  @Override
    public Sendable log() {
        return this;
    }
}