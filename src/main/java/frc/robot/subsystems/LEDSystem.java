package frc.robot.subsystems;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.util.sendable.Sendable;

public class LEDSystem extends LogSubsystem {

    public final Spark blinkin;

    public LEDSystem(){
        blinkin = new Spark(0);
    }


    public void setOutput(double val){
        blinkin.set(DoubleUtils.clamp(val, -1.0, 1.0));
    }

    public void colourOutputShooter(double vel) {
        double targetVel = 1; // subject to change
        double maxDiff = 1; // change
        double diff = Math.abs(vel - targetVel);
        double scaledVal = 0.61 + ((0.77-0.61)*(1 - diff/maxDiff));

        DoubleUtils.clamp(vel, 0.61, 0.77);
        blinkin.set(scaledVal);
    }

    public Sendable log() {
        return this;
    }
}
