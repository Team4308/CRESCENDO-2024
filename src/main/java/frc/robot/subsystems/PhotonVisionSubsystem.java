package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import ca.team4308.absolutelib.wrapper.LogSubsystem;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PhotonVisionSubsystem extends LogSubsystem {
    private Field2d field;
    public PhotonCamera leftCam;
    public PhotonCamera rightCam;
    public PhotonCamera noteCam;
    public PhotonPoseEstimator leftPoseEstimator;
    public PhotonPoseEstimator rightPoseEstimator;

    // Simulation
    private static VisionSystemSim visionSim;
    private static SimCameraProperties cameraProp;
    private static PhotonCameraSim simLeftCam;
    private static PhotonCameraSim simRightCam;

    public enum Cameras {
        LEFT_CAM,
        RIGHT_CAM
    }
    
    public PhotonVisionSubsystem() {
        field = new Field2d();
        leftCam = new PhotonCamera(Constants.Vision.cam1Name);
        rightCam = new PhotonCamera(Constants.Vision.cam2Name);
        noteCam = new PhotonCamera(Constants.Vision.cam3Name);

        leftPoseEstimator = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam, Constants.Vision.kRobotToCam1);
        rightPoseEstimator = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam, Constants.Vision.kRobotToCam2);

        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(Constants.Vision.kTagLayout);

            // Create simulated camera properties. These can be set to mimic your actual camera.
            cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(70));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(30);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            simLeftCam = new PhotonCameraSim(leftCam, cameraProp);
            simLeftCam.enableDrawWireframe(true);

            simRightCam = new PhotonCameraSim(leftCam, cameraProp);
            simRightCam.enableDrawWireframe(true);

            visionSim.addCamera(simLeftCam, Constants.Vision.kRobotToCam1);
            visionSim.addCamera(simRightCam, Constants.Vision.kRobotToCam2);
        }
    }

    public PhotonPipelineResult getLatestResult(Cameras camera) {
        PhotonCamera photonCamera;
        PhotonCameraSim photonCameraSim;
        switch (camera) {
            case LEFT_CAM:
                photonCamera = leftCam;
                photonCameraSim = simLeftCam;
                break;
            case RIGHT_CAM:
                photonCamera = rightCam;
                photonCameraSim = simRightCam;
                break;
            default:
                return null;
        }
        return Robot.isReal() ? photonCamera.getLatestResult() : photonCameraSim.getCamera().getLatestResult();
    }

    public PhotonPoseEstimator getPoseEstimator(Cameras camera) {
        PhotonPoseEstimator photonPoseEstimator;
        switch (camera) {
            case LEFT_CAM:
                photonPoseEstimator = leftPoseEstimator;
                break;
            case RIGHT_CAM:
                photonPoseEstimator = rightPoseEstimator;
                break;
            default:
                return null;
        }
        return photonPoseEstimator;
    }

    public boolean hasTargets(Cameras camera) {
        return getLatestResult(camera).hasTargets();
    }

    public static Pose2d getTagPose(int id) {
        Optional<Pose3d> tag = Constants.Vision.kTagLayout.getTagPose(id);
        if(tag.isPresent()) {
            return tag.get().toPose2d();
        }
        return null;
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
        Optional<EstimatedRobotPose> visionEst = getPoseEstimator(camera).update();
        return visionEst;
    }
    
     /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Cameras camera) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = getLatestResult(camera).getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = getPoseEstimator(camera).getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(getEstimatedGlobalPose(camera).get().estimatedPose.toPose2d().getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    // ----- Simulation
    public void updateVisionField() {
        SmartDashboard.putData("Vision/Field", field);

        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
        if(hasTargets(Cameras.LEFT_CAM)) targets.addAll(getLatestResult(Cameras.LEFT_CAM).targets);
        if(hasTargets(Cameras.RIGHT_CAM)) targets.addAll(getLatestResult(Cameras.RIGHT_CAM).targets);

        List<Pose2d> poses = new ArrayList<>();
        for(PhotonTrackedTarget target : targets) {
            Pose2d targetPose = getTagPose(target.getFiducialId());
            poses.add(targetPose);
        }
        field.getObject("Tracked Targets").setPoses(poses);

        Optional<EstimatedRobotPose> leftPoseEst = getEstimatedGlobalPose(Cameras.LEFT_CAM);
        Optional<EstimatedRobotPose> rightPoseEst = getEstimatedGlobalPose(Cameras.RIGHT_CAM);
        if (leftPoseEst.isPresent()) {
            field.getObject("LeftEstPose").setPose(leftPoseEst.get().estimatedPose.toPose2d());
        }
        if (rightPoseEst.isPresent()) {
            field.getObject("RightEstPose").setPose(rightPoseEst.get().estimatedPose.toPose2d());
        }
    }

    @Override
    public void simulationPeriodic() {
        updateVisionField();
    }

    @Override
    public Sendable log() {
        return this;
    }
}
