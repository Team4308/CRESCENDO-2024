package frc.robot.subsystems.swervedrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Vision {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private final double maximumAmbiguity = 0.25;
    private final PhotonCamera noteCamera;
    
     // Count of times that the odom thinks we're more than 10meters away from the april tag.
    private double longDistangePoseEstimationCount = 0;
    private Supplier<Pose2d> currentPose;
    private Field2d field2d;
    public VisionSystemSim visionSim;

    enum PoseCameras {
        LEFT_CAM("Arduckcam OV9281",
                new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
                new Translation3d(Units.inchesToMeters(12.056),
                        Units.inchesToMeters(10.981),
                        Units.inchesToMeters(8.44)),
                VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
        RIGHT_CAM("Argoosecam OV9281",
                new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
                new Translation3d(Units.inchesToMeters(12.056),
                        Units.inchesToMeters(-10.981),
                        Units.inchesToMeters(8.44)),
                VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

        
        public final Alert latencyAlert;
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;

        // Standard deviations are fake values, experiment and determine estimation noise on an actual robot.
        // https://www.chiefdelphi.com/t/photonvision-finding-standard-deviations-for-swervedriveposeestimator/467802/2
        public final Matrix<N3, N1> singleTagStdDevs;
        public final Matrix<N3, N1> multiTagStdDevs;
        
        // Transform of the camera rotation and translation relative to the center of the robot
        private final Transform3d robotToCamTransform;
        
        public PhotonCameraSim cameraSim;

        PoseCameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
                    Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
            latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.WARNING);
            
            camera = new PhotonCamera(name);
            // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

            poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
                                                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                    camera, robotToCamTransform);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevsMatrix;

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
                // Approximate detection noise with average and standard deviation error in pixels.
                cameraProp.setCalibError(0.25, 0.08);
                // Note: FPS is limited by robot loop rate
                cameraProp.setFPS(30);
                // The average and standard deviation in milliseconds of image data latency.
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);

                cameraSim = new PhotonCameraSim(camera, cameraProp);
                cameraSim.enableDrawWireframe(true);
            }
        }

        // Add camera for simulated PhotonVision
        public void addCameraToVisionSim(VisionSystemSim systemSim) {
            if (Robot.isSimulation()) {
                systemSim.addCamera(cameraSim, robotToCamTransform);
                // cameraSim.enableDrawWireframe(true);
            }
        }
    
    }

    public Vision(Supplier<Pose2d> currentPose, Field2d field) {
        this.currentPose = currentPose;
        this.field2d = field;
        noteCamera = new PhotonCamera("Ardeercam OV9782");

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(fieldLayout);

            for (PoseCameras c : PoseCameras.values()) {
                c.addCameraToVisionSim(visionSim);
            }
        }
    }

    // Calculate target AprilTag pose relative to robot
    public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
        }

    }

    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        if (SwerveDriveTelemetry.isSimulation) {
            visionSim.update(swerveDrive.getPose());
        }
        for (PoseCameras camera : PoseCameras.values()) {
            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
            if (poseEst.isPresent()) {
                var pose = poseEst.get();
                swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                        pose.timestampSeconds,
                        getEstimationStdDevs(camera));
            }
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PoseCameras camera) {
        Optional<EstimatedRobotPose> poseEst = filterPose(camera.poseEstimator.update());
        // Optional<EstimatedRobotPose> poseEst = camera.poseEstimator.update();

        // Uncomment to enable outputting of vision targets in simulation

        // poseEst.ifPresent(estimatedRobotPose -> {
        //     field2d.getObject(camera + "/ Estimated Pose").setPose(estimatedRobotPose.estimatedPose.toPose2d());
        // });
        return poseEst;
    }

    // Standard deviations (trust) of the estimated pose. Only use when targets are visible
    public Matrix<N3, N1> getEstimationStdDevs(PoseCameras camera) {
        var poseEst = getEstimatedGlobalPose(camera);
        var estStdDevs = camera.singleTagStdDevs;
        var targets = getPoseCamLatestResult(camera).getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = camera.poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            if (poseEst.isPresent()) {
                avgDist += PhotonUtils.getDistanceToPose(poseEst.get().estimatedPose.toPose2d(),
                           tagPose.get().toPose2d());
            }
        }
        if (numTags == 0) {
            return estStdDevs;
        }
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = camera.multiTagStdDevs;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return estStdDevs;
    }

    // Filter pose by its ambiguity and find best estimate for poses from long distances
    private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose) {
        if (pose.isPresent()) {
            double bestTargetAmbiguity = 1; // 1 is max ambiguity
            for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                double ambiguity = target.getPoseAmbiguity();
                if (ambiguity != -1 && ambiguity < bestTargetAmbiguity) {
                    bestTargetAmbiguity = ambiguity;
                }
            }
            // If the ambiguity is too high, don't use estimate
            if (bestTargetAmbiguity > maximumAmbiguity) {
                return Optional.empty();
            }

            // Increase count if estimated pose is very far from recorded robot pose
            if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1) {
                longDistangePoseEstimationCount++;

                // If it calculates that we're 10 meter away for more than 10 times in a row, its probably right
                if (longDistangePoseEstimationCount < 10) {
                    return Optional.empty();
                }
            } else {
                longDistangePoseEstimationCount = 0;
            }
            return pose;
        }
        return Optional.empty();
    }

    public PhotonPipelineResult getNoteCamLatestResult() {
        return noteCamera.getLatestResult();
    }

    public PhotonPipelineResult getPoseCamLatestResult(PoseCameras camera) {
        return Robot.isReal() ? camera.camera.getLatestResult() : camera.cameraSim.getCamera().getLatestResult();
    }

    public double getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
    }

    // Get specific target from all visible AprilTags
    public PhotonTrackedTarget getTargetFromId(int id, PoseCameras camera) {
        PhotonTrackedTarget target = null;
        PhotonPipelineResult result = getPoseCamLatestResult(camera);
        if (!result.hasTargets()) {
            return null; 
        }
        for (PhotonTrackedTarget i : result.getTargets()) {
            if (i.getFiducialId() == id) {
                target = i;
            }
        }
        return target;
    }

    public VisionSystemSim getVisionSim() {
        return visionSim;
    }

    // Updates the Vision Field with tracked targets
    public void updateVisionField() {
        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
        for (PoseCameras c : PoseCameras.values()) {
            if (getPoseCamLatestResult(c).hasTargets()) {
                targets.addAll(getPoseCamLatestResult(c).targets);
            }
        }

        List<Pose2d> poses = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
                poses.add(targetPose);
            }
        }

        field2d.getObject("tracked targets").setPoses(poses);
    }

}