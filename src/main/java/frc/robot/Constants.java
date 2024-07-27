package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = 110 * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(10, 0, 0.01);
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class Input {
    public static double kTriggerDeadband = 0.06;
    public static double kJoystickDeadband = 0.06;
  }

  public static class Mapping {
    public static class Intake {
      public static final int intakeMotor = 11;
      public static final int leftBeamBreak = 5;
      public static final int rightBeamBreak = 4;
    }

    public static class Shooter {
      public static final int motor = 13;
      public static final int beambreak = 0;
      public static final int encoder = 1;
      public static final int limitSwitch1 = 2;
      public static final int limitSwitch2 = 3;
    }

    public static class ShooterMotor {
      public static final int kMotor1 = 14;
      public static final int kMotor2 = 15;
    }

    public static class Controllers {
      public static final int kStick = 0;
      public static final int kStick1 = 1;
    }

    public static class ClimbMotors {
      public static final int motor1 = 9;
      public static final int motor2 = 10;
    }

    public static class Index {
      public static final int indexMotor = 12;
    }

    public static class Pigeon2 {
      public static final int gyro = 0;
    }
  }

  public static class Generic {
    public static int timeoutMs = 1000;
  }

  public static class Config {
    public static class Drive {
      public static class Power {
        public static double kOpenLoopRamp = 0.0;
      }

      public static class AngleControl {
        public static double kP = 0.03;
        public static double kI = 0.0;
        public static double kD = 0.01;
      }

      public static class TranslationControl {
        public static double kP = 0.001;
        public static double kI = 0.0;
        public static double kD = 0.1;
      }
    }
  }

  public static class Shooter {
    public static final double shootInAmpMultiplier = 0.5;
    public static final int shooterStartDegree = 18;
    public static final int shooterEndDegree = 72;
    public static final double encoderStartRevolutions = 0.0;
    public static final double encoderEndRevolutions = -0.69;;
    public static final double shooterMaxVelocity = 7;
    public static final double shooterRPS = 100;

    public static final class AngleControl {
      public static final double kP = 0.01;
      public static final double kI = 0;
      public static final double kD = 0.0001;
    }

    public static final class ShooterControl {
      public static final double kV = 0.12;
      public static final double kP = 0.11;
      public static final double kI = 0.48;
      public static final double kD = 0.01;
    }

    public static final class FeedforwardControl {
      public static final double kS = 1.43;
      // Getting kS requires system identification (SysId)
      // Theoratical kS gain using stall torque and moment of inertia
      public static final double kG = 0.43;
      public static final double kV = 2.25;
      public static final double kA = 0.01;
      // Values from ReCalc
    }

    public static final class TrapezoidProfile {
      public static final double kMaxVelocity = 4.45; // in radians / seconds
      public static final double kMaxAcceleration = 27.11; //in radians / seconds ^ 2
      // theoretical values using math, needs to be tuned further
    }
  }

  public static class Limelight {
    public static class Measurements {
      public static final double limelightMountAngleDegrees = 30;
      public static final double limelightLensHeightCM = 22.5;
      public static final double limelightDistanceFromShooterCM = 30.5;
    }
  }

  public static class GamePieces {
    public static class speaker {
      public static final double speakerAprilTagHeightCM = 145.0975;
      // retune speaker opening height for stemley 2024
      public static final double speakerOpeningHeightCM = 205;
      public static final double angle = 60.0;
    }

    public static class amp {
      public static final double angleToshoot = 64;
      public static final double speedToShoot = 14;
    }
  }
}
