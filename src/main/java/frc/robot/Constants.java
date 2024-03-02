package frc.robot;

public final class Constants {
  public static class Mapping {
    public static class Shooter {
      public static final int motor = 1;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static class PID {
    public static class Shooter {
      public static final double kP = 0.03;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }
}
