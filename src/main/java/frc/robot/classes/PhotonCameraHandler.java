package frc.robot.classes;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.configs.cameraConfig;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import java.util.Optional;
import java.util.Vector;

public class PhotonCameraHandler {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final cameraConfig config;

    public PhotonCameraHandler(cameraConfig config) {
        this.config = config;
        this.photonCamera = new PhotonCamera(config.photonCamera);
        this.photonPoseEstimator = new PhotonPoseEstimator(this.config.aprilTagFieldLayout, this.config.photonPoseEstimatorStrategy, this.config.robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        PhotonPipelineResult result = (PhotonPipelineResult) photonCamera.getAllUnreadResults();
        return photonPoseEstimator.update(result);
    }

    public void updateFieldPose(SwerveDrivePoseEstimator poseEstimator) {
        Optional<EstimatedRobotPose> poseEstimate = getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
        if (poseEstimate.isPresent()) {
            Logger.recordOutput("Camera/EstimatedPose", poseEstimate.get().estimatedPose.toPose2d());
            poseEstimator.addVisionMeasurement(poseEstimate.get().estimatedPose.toPose2d(), poseEstimate.get().timestampSeconds);
        }
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }
}
