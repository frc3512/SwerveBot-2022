package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class PhotonVisionWrapper extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator positionEstimation;
    private AprilTagFieldLayout aprilTagLayout;

    public PhotonVisionWrapper() {
        camera = new PhotonCamera(Constants.PhotonVision.photonVisionName);
        aprilTagLayout = new AprilTagFieldLayout(Constants.AprilTags.aprilTagList, FieldConstants.fieldLength, FieldConstants.fieldWidth);
        positionEstimation = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Constants.PhotonVision.robotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        positionEstimation.setReferencePose(prevEstimatedRobotPose);
        return positionEstimation.update();
    }
}