package frc.robot.subsystems;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDSystem extends LogSubsystem {

    public final Spark blinkin;

    public LEDSystem(){
        blinkin = new Spark(0);
    }


    public void setOutput(double val){
        blinkin.set(DoubleUtils.clamp(val, -1.0, 1.0));
    }

    public Sendable log() {
        return this;
    }
}
