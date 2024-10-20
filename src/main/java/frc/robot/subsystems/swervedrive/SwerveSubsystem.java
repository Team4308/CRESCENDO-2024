// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import ca.team4308.absolutelib.wrapper.LoggedTunableNumber;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends LogSubsystem {
  private final SwerveDrive swerveDrive;
  private final Pigeon2 gyro = new Pigeon2(Constants.Mapping.Pigeon2.gyro);
  private final boolean visionDriveTest = false; // Set True for Vision-Correcting Pose Estimation
  
  private Vision vision;
  private double modifier = 1.0;
  private boolean resetHeading = false;
  public DriveMode driveMode = DriveMode.VELOCITY_ADV;

  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public double maximumSpeed = Units.feetToMeters(15.1);

  private static final LoggedTunableNumber kAngleP = new LoggedTunableNumber("Swerve/Align/kAngleP",
      Constants.Swerve.AngleControl.kP);
  private static final LoggedTunableNumber kAngleI = new LoggedTunableNumber("Swerve/Align/kAngleI",
      Constants.Swerve.AngleControl.kI);
  private static final LoggedTunableNumber kAngleD = new LoggedTunableNumber("Swerve/Align/kAngleD",
      Constants.Swerve.AngleControl.kD);

  private static final LoggedTunableNumber kTranslationP = new LoggedTunableNumber("Swerve/Align/kTranslationP",
      Constants.Swerve.TranslationControl.kP);
  private static final LoggedTunableNumber kTranslationI = new LoggedTunableNumber("Swerve/Align/kTranslationI",
      Constants.Swerve.TranslationControl.kI);
  private static final LoggedTunableNumber kTranslationD = new LoggedTunableNumber("Swerve/Align/kTranslationD",
      Constants.Swerve.TranslationControl.kD);

  private PIDConstants ANGLE_CONTROLLER;
  private PIDConstants TRANSLATION_CONTROLLER;

  public enum DriveMode {
    /** Driving with input from driver joysticks. (Default) */
    VELOCITY_ADV,
    ABSOLUTE,
    ROBOT_RELATIVE,
    SPEAKER,
    AMP,
    NOTE
  }

  public SwerveSubsystem(File directory) {
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);

    // Change Telemetry to reduce Shuffleboard vomit. (Note: Use HIGH for simulations)
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Swerve.MAX_SPEED);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setHeadingCorrection(false); 
    // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

    if (visionDriveTest) {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    ANGLE_CONTROLLER = new PIDConstants(kAngleP.get(), kAngleI.get(), kAngleD.get());
    TRANSLATION_CONTROLLER = new PIDConstants(kTranslationP.get(), kTranslationI.get(), kTranslationD.get());
    
    setupPathPlanner();
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.Swerve.MAX_SPEED);
  }

  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  @Override
  public void periodic() {
    // Updates odometry with pose correction if vision is enabled 
    if (visionDriveTest)
    {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }

    if (DriverStation.isEnabled()) {
      LoggedTunableNumber.ifChanged(
          hashCode(), () -> ANGLE_CONTROLLER = new PIDConstants(kAngleP.get(), kAngleI.get(), kAngleD.get()),
                                                                kAngleP, kAngleI, kAngleD);
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () -> TRANSLATION_CONTROLLER = new PIDConstants(kTranslationP.get(), kTranslationI.get(), kTranslationD.get()), 
                                                          kTranslationP, kTranslationI, kTranslationD);
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  // Set up AutoBuilder for PathPlanner
  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Will be called if auto has a starting pose
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            TRANSLATION_CONTROLLER,
            ANGLE_CONTROLLER,
            4.5,
            // Note: in m/s
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()
            // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public double getDistanceToSpeaker() {
    Pose3d speakerCenterPose = new Pose3d();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      speakerCenterPose = Constants.GamePieces.Speaker.kSpeakerCenterBlue;
    } else {
      speakerCenterPose = Constants.GamePieces.Speaker.kSpeakerCenterRed;
    }
    //int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    //Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
    return getPose().getTranslation().getDistance(speakerCenterPose.toPose2d().getTranslation());
  }

  public Rotation2d getSpeakerYaw() {
    int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
    Translation2d relativeTrl = speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
  }

  public Rotation2d getAmpYaw() {
    int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 5;
    Pose3d ampAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
    Translation2d relativeTrl = ampAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
  }

  public Command aimAtSpeaker(double tolerance) {
    SwerveController controller = swerveDrive.getSwerveController();
    return run(
        () -> {
          drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,
              controller.headingCalculate(getHeading().getRadians(),
              getSpeakerYaw().getRadians()),
              getHeading()));
        }).until(() -> getSpeakerYaw().minus(getHeading()).getDegrees() < tolerance);
  }

  public Command aimAtTarget(PhotonCamera camera) {
    return run(() -> {
      PhotonPipelineResult result = camera.getLatestResult();
      if (result.hasTargets()) {
        drive(getTargetSpeeds(0, 0, 
        Rotation2d.fromDegrees(result.getBestTarget().getYaw()))); 
        // Not sure if this will work, more math may be required. - nstrike
      }
    });
  }

  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  // Create a path following command using AutoBuilder. This will also trigger event markers
  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  // Use PathPlanner Path finding to go to a point on the field.
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 4.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
  }

  /**
   * Command to drive the robot using translative values and heading as either angular
   * velocity or absolute heading as a setpoint along with presets to snap the robot 
   * towards directions and aligning to game pieces.
   * 
   * @param translationX     Translation in the X direction of Left Joystick.
   * @param translationY     Translation in the Y direction of Left Joystick.
   * @param rotationX        Rotation in the X direction of Right Joystick.
   * @param rotationY        Rotation in the Y direction of Right Joystick.
   * @param lookAway         Face the robot towards the opposing alliance's wall
   * @param lookTowards      Face the robot towards the driver
   * @param lookLeft         Face the robot left
   * @param lookRight        Face the robot right
   * @param driveMode        Driving Modes
   * @return Drive command.
   */

  public Command drivePresetsCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationX, DoubleSupplier rotationY, BooleanSupplier lookAway, 
      BooleanSupplier lookTowards, BooleanSupplier lookLeft, BooleanSupplier lookRight) {
    return run(() -> {
      switch (driveMode) {
        case VELOCITY_ADV:
          driveVelocityAdvanced(translationX, translationY, rotationX, lookAway, 
                      lookTowards, lookLeft, lookRight, true);
          break;
        case ABSOLUTE:
          driveAbsolute(translationX, translationY, rotationX, rotationY, true);
          break;
        case ROBOT_RELATIVE:
          driveVelocityAdvanced(translationX, translationY, rotationX, lookAway, 
                      lookTowards, lookLeft, lookRight, false);
          break;
        case SPEAKER:
          alignToSpeaker(translationX, translationY);
          break;
        case AMP:
          alignToAmp(translationX, translationY);
          break;
        case NOTE:
          alignToNote(translationX, translationY);
          break;
        default:
          break;
      }
    });
  }

  public void driveVelocityAdvanced(DoubleSupplier translationX, DoubleSupplier translationY, 
                                    DoubleSupplier angularRotationX, BooleanSupplier lookAway, 
                                    BooleanSupplier lookTowards, BooleanSupplier lookLeft, BooleanSupplier lookRight, 
                                    boolean fieldRelative) {
    double headingX = 0;
    double headingY = 0;

    // Face Away from Drivers
    if (lookAway.getAsBoolean()) {
      headingY = -1;
    }
    // Face Right
    if (lookRight.getAsBoolean()) {
      headingX = 1;
    }
    // Face Left
    if (lookLeft.getAsBoolean()) {
      headingX = -1;
    }
    // Face Towards the Drivers
    if (lookTowards.getAsBoolean()) {
      headingY = 1;
    }

    // Prevent Movement After Auto
    if (resetHeading == true) {
      if (headingX == 0 && headingY == 0 && Math.abs(angularRotationX.getAsDouble()) == 0) {
        // Get the curret Heading
        Rotation2d currentHeading = getHeading();

        // Set the Current Heading to the desired Heading
        headingX = currentHeading.getSin();
        headingY = currentHeading.getCos();
      }
      // Dont reset Heading Again
      resetHeading = false;
    }

    ChassisSpeeds desiredSpeeds = getTargetSpeeds(translationX.getAsDouble(), translationY.getAsDouble(),
                                                  headingX, headingY);
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

    if (headingX == 0 && headingY == 0 && Math.abs(angularRotationX.getAsDouble()) > 0) {
      resetHeading = true;
      swerveDrive.drive(SwerveMath.scaleTranslation(translation, modifier), 
                      angularRotationX.getAsDouble() * 
                      swerveDrive.getMaximumAngularVelocity() * modifier,
                      fieldRelative, false);
    } else {
      swerveDrive.drive(SwerveMath.scaleTranslation(translation, modifier),
                        desiredSpeeds.omegaRadiansPerSecond,
                        fieldRelative,false);
    }
  }

  public void driveAbsolute(DoubleSupplier translationX, DoubleSupplier translationY, 
                            DoubleSupplier headingX, DoubleSupplier headingY, boolean fieldRelative) {
    
    ChassisSpeeds desiredSpeeds = getTargetSpeeds(translationX.getAsDouble(), translationY.getAsDouble(), 
                                                          headingX.getAsDouble(), headingY.getAsDouble());
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    swerveDrive.drive(SwerveMath.scaleTranslation(translation, modifier),
                            desiredSpeeds.omegaRadiansPerSecond * modifier,
                            fieldRelative, false);
  }

  // Drive method with heading aligned to the speaker
  public void alignToSpeaker(DoubleSupplier translationX, DoubleSupplier translationY) {
    SwerveController controller = swerveDrive.getSwerveController();
    double rotation =  controller.headingCalculate(getHeading().getRadians(), getSpeakerYaw().getRadians());
    if (Math.abs(getSpeakerYaw().minus(getHeading()).getDegrees()) <= 1.0) {
      rotation = 0;
    }
    swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                      translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                      translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), modifier),
                      rotation,true, false);
  }

  // Drive method with heading aligned to the amp
  public void alignToAmp(DoubleSupplier translationX, DoubleSupplier translationY) {
    ChassisSpeeds desiredSpeeds = getTargetSpeeds(translationX.getAsDouble(), translationY.getAsDouble(),
                                                  -1, 0);
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

    swerveDrive.drive(SwerveMath.scaleTranslation(translation, modifier),
                        desiredSpeeds.omegaRadiansPerSecond,
                        true,false);
  }

  // Drive method with heading aligned to a note
  public void alignToNote(DoubleSupplier translationX, DoubleSupplier translationY) {
    SwerveController controller = swerveDrive.getSwerveController();
    double rotation =  controller.headingCalculate(getHeading().getRadians(), getSpeakerYaw().getRadians());
    if (Math.abs(getSpeakerYaw().minus(getHeading()).getDegrees()) <= 1.0) {
      rotation = 0;
    }
    swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                      translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                      translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), modifier),
                      rotation,true, false);
  }  

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity. "Velocity Drive"
   * 
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Angular velocity of the robot to set.
   * @return Drive command.
   */
  public Command driveVelocityCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      double transX = translationX.getAsDouble();
      double transY = translationY.getAsDouble();
      if (DriverStation.getAlliance().get() == Alliance.Red ) {
        transX *= -1.0;
        transY *= -1.0;
      }
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            transX * swerveDrive.getMaximumVelocity(),
                            transY * swerveDrive.getMaximumVelocity()), modifier),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity()
                         * modifier, true,false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint. "Absolute Drive"
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveAbsoluteCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {
      Vector2 driveInput = new Vector2(translationX.getAsDouble(), translationY.getAsDouble());
      driveInput = JoystickHelper.scaleStick(driveInput, 2);
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(driveInput.x, driveInput.y,
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint for simulation.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(() -> {
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
          translationY.getAsDouble(),
          rotation.getAsDouble() * Math.PI,
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  // Commands to characterize the robot drive and angle motors using SysId
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12),
        3.0, 5.0, 3.0);
  }

  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  // public double getOffsetAngleLeftRight() {
  // double targetOffsetAngle_Vertical = LimelightHelpers.getTY("");
  // double targetOffsetAngle_Horizontal = LimelightHelpers.getTX("");
  // double limelightMountAngleDegrees =
  // Constants.Limelight.Measurements.limelightMountAngleDegrees;
  // double limelightLensHeightCM =
  // Constants.Limelight.Measurements.limelightLensHeightCM;
  // double goalHeightCM = Constants.GamePieces.speaker.speakerAprilTagHeightCM;
  // double angleToGoalDegrees = limelightMountAngleDegrees +
  // targetOffsetAngle_Vertical;
  // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  // double distanceFromLimelightToGoalCM = (goalHeightCM - limelightLensHeightCM)
  // / Math.tan(angleToGoalRadians);
  // double botAngle = gyro.getAngle() % 360;

  // double Vs = Constants.Shooter.shooterMaxVelocity; // shooter velocity
  // double dY = distanceFromLimelightToGoalCM * Math.cos((botAngle) * (3.14159 /
  // 180.0)) / 100; // y distance from
  // // speaker
  // double dX = distanceFromLimelightToGoalCM * Math.sin((botAngle +
  // targetOffsetAngle_Horizontal) * (3.14159 / 180.0))
  // / 100; // x distance from speaker
  // double vrX = getFieldVelocity().vxMetersPerSecond; // horziontal velocity to
  // speaker
  // double vrY = getFieldVelocity().vyMetersPerSecond; // vertical velocity to
  // speaker

  // double fracTop = Math.sqrt(-1 * vrY * vrY * dX * dX + 2 * vrY * dY * vrX * dY
  // - dY * dY * vrX * vrX
  // + dY * dY * Vs * Vs + Vs * Vs * dX * dX) - dY * Vs;
  // double fracBottom = (-1 * vrY * dX + dY * vrX + Vs * dX);

  // if (180 < botAngle && botAngle <= 360) {
  // botAngle = botAngle - 360;
  // } else if (-360 <= botAngle && botAngle < -180) {
  // botAngle = botAngle + 360;
  // }

  // return -DoubleUtils.clamp(
  // angle_controller.calculate(botAngle, 2 * Math.atan(fracTop / fracBottom) *
  // (180.0 / 3.14159)), -3 * Math.PI,
  // 3 * Math.PI);
  // }

   // The primary method for controlling the drivebase using the SwerveDrive class
   // It is reccommended to use this method isntead of the other two below.
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  // Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this method. 
  // However, if either gyro angle or module position is reset, this must called in order for odometry to keep working.
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  // Gets the current pose (position and rotation) of the robot, as reported by odometry. 
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  // Set chassis speeds with closed-loop velocity control.
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  // Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
  
  
  // Checks if the alliance is red, defaults to false if alliance isn't available.
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  // Zero the robot to assume current position is facing forward
  // If red alliance, rotate the robot 180 after the drivebase zero command
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  // Sets the drive motors to brake/coast mode.
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  // Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
  // Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  // Get the chassis speeds based on controller input of 2 joysticks. 
  // One for speeds in which direction, the other for the angle of the robot.
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.Swerve.MAX_SPEED);
  }

  // Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of 90deg.
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.Swerve.MAX_SPEED);
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /*
   * The below are getters and setters for 4308 state machines
   */

  public void setDriveMode(DriveMode newDriveMode) {
    this.driveMode = newDriveMode;
  }

  public void setModifier(Double value) {
    modifier = value;
  }

  public Command changeDriveMode(DriveMode newDriveMode) {
    return this.runOnce(() -> setDriveMode(newDriveMode));
  }

  public Command setModifierCommand(Double value) {
    return this.runOnce(() -> setModifier(value));
  }

  @Override
  public Sendable log() {
    SmartDashboard.putString("SwerveDriveVelocity", getRobotVelocity().toString());
    SmartDashboard.putString("Drive Mode", driveMode.toString());
    SmartDashboard.putBoolean("resetHeading", resetHeading);
    return this;
  }
}
