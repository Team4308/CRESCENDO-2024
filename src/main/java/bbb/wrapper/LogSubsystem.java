package bbb.wrapper;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LogSubsystem extends SubsystemBase {
    public abstract Sendable log();

    public abstract void stopControllers();
}