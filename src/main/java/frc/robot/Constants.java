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

  public static class Shooter {
    public static final int shooterStartDegree = 16;
    public static final int shooterEndDegree = 43;
    public static final int motorStartRevolutions = 0;
    public static final double motorEndRevolutions = 14.25925757;
  }
  
  public static class PID {
    public static class Shooter {
      public static final double kP = 0.03;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }

  public static class Limelight {
    public static class measurements {
      public static final double limelightMountAngleDegrees = 25;
      public static final double limelightLensHeightCM = 20;
    }
  }

  public static class gamePieces {
    public static class dimensions {
      public static final double stageHeightCM = 60.0;
    }
  }
}
