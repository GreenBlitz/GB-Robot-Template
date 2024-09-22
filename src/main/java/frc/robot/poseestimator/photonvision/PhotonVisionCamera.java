package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.utils.GBSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class PhotonVisionCamera extends GBSubsystem {

	private final PhotonCamera camera;
	private final Transform3d cameraToRobot;
	private final PhotonVisionTarget target;

	public PhotonVisionCamera(String logPath, String cameraName, Transform3d cameraToRobot, PhotonVisionTarget target) {
		super(logPath + cameraName + "/");
		this.camera = new PhotonCamera(cameraName);
		this.cameraToRobot = cameraToRobot;
		this.target = target;
	}

	public PhotonVisionCamera(CameraConfiguration cameraConfiguration) {
		this(
			cameraConfiguration.logPathPrefix(),
			cameraConfiguration.name(),
			cameraConfiguration.cameraToRobot(),
			cameraConfiguration.targetType()
		);
	}

	public Optional<PhotonPoseData> getBestTargetData() {
		PhotonPipelineResult pipelineResult = camera.getLatestResult();
		PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
		Optional<Pose3d> targetPose = calculateTargetPose(bestTarget, target);
		if (targetPose.isEmpty()) {
			return Optional.empty();
		}
		double latency = pipelineResult.getLatencyMillis();
		double ambiguity = bestTarget.getPoseAmbiguity();
		double timestamp = pipelineResult.getTimestampSeconds();
		return Optional.of(new PhotonPoseData(targetPose.get(), target, timestamp, ambiguity, latency));
	}

	private Optional<Pose3d> calculateTargetPose(PhotonTrackedTarget bestTarget, PhotonVisionTarget target) {
		return switch (target) {
			case APRIL_TAG -> calculateRobotPoseToField(bestTarget);
			case GAME_OBJECT -> calculateGameObjectPoseToRobot(bestTarget);
		};
	}

	private Optional<Pose3d> calculateGameObjectPoseToRobot(PhotonTrackedTarget bestTarget) {
		Transform3d gameObjectToCamera = bestTarget.getBestCameraToTarget().inverse();
		Transform3d gameObjectToRobot = gameObjectToCamera.plus(cameraToRobot);
		Pose3d notePose = new Pose3d(gameObjectToRobot.getTranslation(), gameObjectToRobot.getRotation());
		return Optional.of(notePose);
	}

	private Optional<Pose3d> calculateRobotPoseToField(PhotonTrackedTarget bestTarget) {
		Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
		Optional<Pose3d> tagPoseInField = PhotonVisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(bestTarget.getFiducialId());
		if (tagPoseInField.isEmpty()) {
			return Optional.empty();
		}
		Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPoseInField.get(), cameraToRobot);
		return Optional.of(robotPose);
	}

	@Override
	protected void subsystemPeriodic() {}

}
