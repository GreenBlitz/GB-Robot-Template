package frc.utils.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class PhotonCameraPoseSource {

    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;


    protected PhotonCameraPoseSource(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
        this.photonCamera = camera;
        this.photonPoseEstimator = poseEstimator;
    }

    public Optional<Pose3d> getEstimatedPose(){
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(getCameraResult());
        return estimatedPose.map(estimatedRobotPose -> estimatedRobotPose.estimatedPose);
    }

    public PhotonPipelineResult getCameraResult (){
        return photonCamera.getLatestResult();
    }

    public void update(){
        photonPoseEstimator.update(photonCamera.getLatestResult());
    }




}
