package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.poseestimator.photonvision.photoncamera.GBPhotonCamera;
import frc.robot.poseestimator.photonvision.photoncamera.PhotonTargetData;

import java.util.ArrayList;
import java.util.Optional;

public class PhotonVisionObservationFiltered {

	private ArrayList<GBPhotonCamera> cameras = new ArrayList<>();

	public PhotonVisionObservationFiltered(String logPath) {
        for (CameraConfiguration cameraConfiguration : PhotonConstants.CAMERAS_CONFIGURATION) {
            cameras.add(new GBPhotonCamera(cameraConfiguration));
        }
    }

    private boolean isDataAmbiguous(PhotonTargetData targetData) {
        return targetData.ambiguity() >= PhotonConstants.MAXIMUM_AMBIGUOUS;
    }
    
    private boolean isDataLatencyTooHigh(PhotonTargetData targetData) {
        return targetData.latency() >= PhotonConstants.MAXIMUM_LATENCY;
    }

    private boolean keepLimelightData(PhotonTargetData targetData) {
        return !isDataAmbiguous(targetData) && !isDataLatencyTooHigh(targetData);
    }

    public ArrayList<PhotonTargetData> getAllTargetData() {
        ArrayList<PhotonTargetData> output = new ArrayList<>();
        for (GBPhotonCamera camera : cameras) {
            Optional<PhotonTargetData> bestTarget = camera.getBestTargetData();
            bestTarget.ifPresent(output::add);
        }

        return output;
    }

    public ArrayList<VisionObservation> getAllFilteredObservations() {
        ArrayList<VisionObservation> output = new ArrayList<>();

        for (PhotonTargetData targetData : getAllTargetData()) {
            Pose3d robot3DPose = targetData.robotPose();
            Pose2d robot2DPose = new Pose2d(
                    robot3DPose.getX(),
                    robot3DPose.getY(),
                    robot3DPose.getRotation().toRotation2d()
            );

            VisionObservation observation = new VisionObservation(
                    robot2DPose,
                    getStandardDeviations(targetData),
                    targetData.timestamp()
            );

            output.add(observation);
        }
    }

    public double[] getStandardDeviations(PhotonTargetData targetData) {
        double ambiguity = targetData.ambiguity();
        return new double[] {
                ambiguity / PhotonConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
                ambiguity / PhotonConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
                ambiguity / PhotonConstants.AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR
        };
    }



}
