package frc.robot.poseestimator.photonvision.photoncamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.poseestimator.photonvision.PhotonConstants;
import frc.robot.poseestimator.photonvision.PhotonTarget;
import frc.utils.GBSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class GBPhotonCamera extends GBSubsystem {

	private final PhotonCamera camera;
	private final Transform3d cameraToRobot;
	private final PhotonTarget target;

	public GBPhotonCamera(Transform3d cameraToRobot, String cameraName, PhotonTarget target) {
		super(cameraName + "/");
		this.camera = new PhotonCamera(cameraName);
		this.cameraToRobot = cameraToRobot;
		this.target = target;
	}

	public Optional<PhotonTargetData> getBestTargetData() {
		PhotonPipelineResult pipelineResult = camera.getLatestResult();
		PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
		Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
		Optional<Pose3d> fieldInTag = PhotonConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(bestTarget.getFiducialId());
		if (!fieldInTag.isPresent()) {
			return Optional.empty();
		}

		Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, fieldInTag.get(), cameraToRobot);
		double latency = pipelineResult.getLatencyMillis();
		double ambiguity = bestTarget.getPoseAmbiguity();

		double timestamp = pipelineResult.getTimestampSeconds();

		return Optional.of(new PhotonTargetData(robotPose, target, timestamp, ambiguity, latency));
	}

	@Override
	protected void subsystemPeriodic() {}

}
