package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;

public class PhotonVision {
    PhotonCamera aprilTag1 = new PhotonCamera("Apriltag_1");
    
    // location of camera on the robot
    public final Transform3d aprilTag1Pos = new Transform3d(new Translation3d(), new Rotation3d());

    public final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTag1Pos);

    public PhotonVision() {

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Optional<EstimatedRobotPose> getRobotPose() {
    Optional<EstimatedRobotPose> robotPose = Optional.empty();

    for (PhotonPipelineResult change : aprilTag1.getAllUnreadResults()){
        robotPose = poseEstimator.update(change);
    }

    return robotPose;
    }


  
}
