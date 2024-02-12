package frc.robot;
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Mapping {
    public static class ShooterMotor {
      public static int kMotor1 = 0;
      public static int kMotor2 = 0;
    }
    public static class Controllers {
      public static int kStick1 = 0;
      public static int kStick2 = 1;
    }
  }
  
  public static class Generic {
    public static int kTimeoutMs = 1000;
  }

  public static class Config {
    public static class Input {
      public static double kInputDeadband = 0.14;

      public static class Stick {
        public static double kInputScale = 2;
      }
    }
  }
}
