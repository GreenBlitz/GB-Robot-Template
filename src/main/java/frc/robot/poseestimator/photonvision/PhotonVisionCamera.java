package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.utils.GBSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PhotonVisionCamera extends GBSubsystem {

	private final PhotonCamera camera;
	private final Transform3d cameraToRobot;
	private final PhotonVisionTarget targetType;

	public PhotonVisionCamera(String cameraName, Transform3d cameraToRobot, PhotonVisionTarget target) {
		super(PhotonVisionConstants.camerasLogPathPrefix + cameraName + "/");
		super.setName("PhotonVisionCamera " + cameraName);
		this.camera = new PhotonCamera(cameraName);
		this.cameraToRobot = cameraToRobot;
		this.targetType = target;
	}

	public PhotonVisionCamera(CameraConfiguration cameraConfiguration) {
		this(cameraConfiguration.name(), cameraConfiguration.cameraToRobot(), cameraConfiguration.targetType());
	}

	public Optional<PhotonVisionTargetRawData>
		tackedTargetToTargetRawdata(PhotonTrackedTarget trackedTarget, PhotonPipelineResult pipelineResult) {
		double latency = pipelineResult.getLatencyMillis();
		double ambiguity = trackedTarget.getPoseAmbiguity();
		double timestamp = pipelineResult.getTimestampSeconds();

		Optional<Pose3d> targetPose = calculateTargetPose(trackedTarget);
		return targetPose.map(pose3d -> new PhotonVisionTargetRawData(camera.getName(), pose3d, timestamp, ambiguity, latency));
	}

	public Optional<PhotonVisionTargetRawData> getBestTargetData() {
		PhotonPipelineResult pipelineResult = camera.getLatestResult();
		Optional<PhotonTrackedTarget> bestTarget = Optional.of(pipelineResult.getBestTarget());
		if (bestTarget.isEmpty()) {
			return Optional.empty();
		}
		return tackedTargetToTargetRawdata(bestTarget.get(), pipelineResult);
	}

	public Optional<ArrayList<Optional<PhotonVisionTargetRawData>>> getTargetsData() {
		PhotonPipelineResult pipelineResult = camera.getLatestResult();
		Optional<List<PhotonTrackedTarget>> targets = Optional.of(pipelineResult.getTargets());
		ArrayList<Optional<PhotonVisionTargetRawData>> output = new ArrayList<>();

		if (targets.isEmpty()) {
			return Optional.empty();
		}

		for (PhotonTrackedTarget target : targets.get()) {
			output.add(tackedTargetToTargetRawdata(target, pipelineResult));
		}

		return Optional.of(output);
	}

	private Optional<Pose3d> calculateTargetPose(PhotonTrackedTarget bestTarget) {
		return switch (targetType) {
			case GAME_OBJECT -> calculateGameObjectPoseToRobot(bestTarget);
			case APRIL_TAG -> calculateRobotPoseToField(bestTarget);
		};
	}

	private Optional<Pose3d> calculateGameObjectPoseToRobot(PhotonTrackedTarget bestTarget) {
		Transform3d gameObjectToCamera = bestTarget.getBestCameraToTarget().inverse();
		Transform3d gameObjectToRobot = gameObjectToCamera.plus(cameraToRobot);
		Pose3d gameObjectPose = new Pose3d(gameObjectToRobot.getTranslation(), gameObjectToRobot.getRotation());
		return Optional.of(gameObjectPose);
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