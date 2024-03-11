package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class pigeon2System extends LogSubsystem {

    Pigeon2 pigeon = new Pigeon2(Constants.Mapping.Pigeon2.gyro);

    public pigeon2System(){
        
    }

    public double getAccelerationX(){
        return pigeon.getAccelerationX().getValueAsDouble();
    }

    public double getAccelerationY(){
        return pigeon.getAccelerationY().getValueAsDouble();
    }

    public double getAngle() {
        return pigeon.getAngle();
    }

    public Sendable log() {
        return this;
    }

    
}
