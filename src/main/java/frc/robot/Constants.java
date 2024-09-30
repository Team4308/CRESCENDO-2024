package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import swervelib.math.Matter;

public final class Constants {
  public static final class LoggedDashboard{
    public static final boolean tuningMode = true; 
  }

  public static class Swerve {
    public static final double ROBOT_MASS = 110 * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static class Auton {
      public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
      public static final PIDConstants ANGLE_PID = new PIDConstants(10, 0, 0.01);
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
  
  public static class Vision {
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        // NOT USED CURRENTLY
        // https://www.chiefdelphi.com/t/photonvision-finding-standard-deviations-for-swervedriveposeestimator/467802/2
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class Controller {
    public static class Driver {
      public static final double LEFT_X_DEADBAND = 0.1;
      public static final double LEFT_Y_DEADBAND = 0.1;
      public static final double RIGHT_X_DEADBAND = 0.1;
      public static final double RIGHT_Y_DEADBAND = 0.1;
      public static final double TURN_CONSTANT = 6;
    }
    public static class Operator {
      public static final double TRIGGER_DEADBAND = 0.06;
      public static final double JOYSTICK_DEADBAND = 0.06;
    }
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
      public static final int driver = 0;
      public static final int operator = 1;
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

  public static class Shooter {
    public static final double shootInAmpMultiplier = 0.5;
    public static final double shooterStartDegree = 18;
    public static final double shooterEndDegree = 72;
    public static final double encoderStartRevolutions = 0.0;
    public static final double encoderEndRevolutions = -0.69;;
    public static final double shooterMaxVelocity = 7;
    public static final double shooterRPS = 100;

    public static final class PivotPID {
      public static final double kP = 0.01;
      public static final double kI = 0;
      public static final double kD = 0.0001;
    }

    public static final class FlywheelControl {
      public static final double kV = 0.12;
      public static final double kP = 0.11;
      public static final double kI = 0.48;
      public static final double kD = 0.01;
    }

    public static final class PivotFF {
      public static final double kS = 1.43;
      // Values from ReCalc
      public static final double kG = 0.43;
      public static final double kV = 2.25;
      public static final double kA = 0.01;
    }

    public static final class TrapezoidProfile {
      public static final double kMaxVelocity = 200; // degrees
      public static final double kMaxAcceleration = 100; //degrees
      // should be safe enough values? needs to be tuned
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
    public static class Speaker {
      public static final double speakerAprilTagHeightCM = 145.0975;
      public static final Pose3d kSpeakerCenterBlue = new Pose3d(0.2167, 5.549, 2.12, new Rotation3d());
      public static final Pose3d kSpeakerCenterRed = new Pose3d(16.3, 5.549, 2.12, new Rotation3d());
      // Retune for STEMLEY
      public static final double speakerOpeningHeightCM = 205;
      public static final double angle = 60.0;
    }

    public static class Amp {
      public static final double angleToshoot = 64;
      public static final double speedToShoot = 14;
    }
  }
}
