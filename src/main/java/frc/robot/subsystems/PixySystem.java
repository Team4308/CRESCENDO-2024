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


public class PixySystem extends SubsystemBase {
  private Pixy2 pixy;
  private int targetCount;
  private int sig = Pixy2CCC.CCC_SIG1;
  private ArrayList<Block> targets;

  public PixySystem() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();
    pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
		pixy.setLED(255, 255, 255); // Sets the RGB LED to full white
    targets = new ArrayList<Block>();
  }

  @Override
  public void periodic() {
    targetCount = pixy.getCCC().getBlocks(false, sig, 8);
    SmartDashboard.putNumber("Targets found", targetCount);
    targets = pixy.getCCC().getBlockCache();
    findClosestTarget();
    // This method will be called once per scheduler run
  }

  public int[] findClosestTarget(){
    int[] directionSize = new int[2]; // index 0 is amount from center of view from -157 to 157, 1 is width of the target
    if(targetCount == 0){
      directionSize[0] = 200; //set outside range to know the target is not in view
      return directionSize;
    }
    // searching for the largest target in view which should be the closest
    Block closestTarget = targets.get(0);
    for(int index = 1; index<targets.size(); index++){
      if(targets.get(index).getWidth()>closestTarget.getWidth()){
        closestTarget = targets.get(index);
      }
    }
    directionSize[0]=closestTarget.getX()-157;//setting the center of the camera view to be center.  
    directionSize[1]=closestTarget.getWidth();
    SmartDashboard.putNumber("Largest x location", directionSize[0]);
    SmartDashboard.putNumber("Largest width", directionSize[1]);
    return directionSize; 
  }

}