package frc.robot.vision.cameras.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.vision.cameras.photon.inputs.PhotonInputsSet;
import frc.robot.vision.interfaces.IndependentRobotPoseSupplier;
import frc.utils.math.StandardDeviations2D;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;
import java.util.function.Supplier;

public final class Photon implements IndependentRobotPoseSupplier {

	private final PhotonCamera camera;

	private final PhotonInputsSet inputs;
	private final Pose3d cameraPoseRobotRelative;
	private final String logPath;
	private final Predicate<PhotonPipelineResult> filter;
	private final AprilTagFieldLayout fieldLayout;

	private Supplier<StandardDeviations2D> stdDevsSupplier;

	public Photon(String cameraName, Pose3d cameraPoseRobotRelative, String logPath, AprilTagFieldLayout fieldLayout) {
		this.camera = new PhotonCamera(cameraName);

		this.inputs = new PhotonInputsSet();
		this.cameraPoseRobotRelative = cameraPoseRobotRelative;
		this.logPath = logPath + "/" + cameraName;
		this.filter = (r) -> true;
		this.stdDevsSupplier = () -> PhotonConstants.DEFAULT_STDDEV_FUNCTION_BY_AMBIGUITY.apply(inputs.aprilTagDetectionInputs().poseAmbiguity);
		this.fieldLayout = fieldLayout;

		Logger.recordOutput(logPath + "/fieldLayout", fieldLayout.getClass().getName());
	}

	private double getTimestamp() {
		return inputs.aprilTagDetectionInputs().timestamp;
	}

	private Pose2d getBestTargetPose() {
		return inputs.aprilTagDetectionInputs().robotPoseAccordingToBestTarget.toPose2d();
	}

	private StandardDeviations2D getStdDevs() {
		return stdDevsSupplier.get();
	}

	public void setStdDevsSupplier(Supplier<StandardDeviations2D> stdDevsSupplier) {
		this.stdDevsSupplier = stdDevsSupplier;
	}

	public Supplier<StandardDeviations2D> getStdDevsSupplier() {
		return stdDevsSupplier;
	}


	@Override
	public Optional<RobotPoseObservation> getIndependentRobotPose() {
		if (inputs.aprilTagDetectionInputs().hasTargets) {
			Pose2d estimatedRobotPose = getBestTargetPose();
			Logger.recordOutput(logPath + "/estimated2DPose", estimatedRobotPose);
			return Optional.of(new RobotPoseObservation(getTimestamp(), estimatedRobotPose, getStdDevs(), camera.getName()));
		}
		return Optional.empty();
	}

	private void updateHardwareInputs() {
		inputs.hardwareInputs().connected = camera.isConnected();
		inputs.hardwareInputs().driverMode = camera.getDriverMode();
		inputs.hardwareInputs().FPSLimit = camera.getFPSLimit();

		Logger.processInputs(logPath + "/hardware", inputs.hardwareInputs());
	}

	private void updateAprilTagDetectionInputs() {
		List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
		Optional<PhotonPipelineResult> optionalResult = processResults(unreadResults, filter);
		boolean seesTarget = optionalResult.isPresent();
		inputs.aprilTagDetectionInputs().hasTargets = seesTarget;

		if (!seesTarget) {
			return;
		}

		PhotonPipelineResult result = optionalResult.get();
		PhotonTrackedTarget bestTarget = result.getBestTarget();
		Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
		Optional<Pose3d> tagPoseFieldRelative = fieldLayout.getTagPose(bestTarget.getFiducialId());
		if (tagPoseFieldRelative.isEmpty()) {
			Logger.recordOutput(logPath + "/aprilTagDetection/unknownFiducialId", bestTarget.getFiducialId());
			return;
		}
		Transform3d cameraToRobotTransform = new Transform3d(cameraPoseRobotRelative.getTranslation(), cameraPoseRobotRelative.getRotation());

		Pose3d robotFieldRelative = tagPoseFieldRelative.get() // tag field-relative
			.transformBy(cameraToTarget.inverse()) // camera field-relative
			.transformBy(cameraToRobotTransform.inverse());

		inputs.aprilTagDetectionInputs().robotPoseAccordingToBestTarget = robotFieldRelative;
		inputs.aprilTagDetectionInputs().bestTargetFiducialId = bestTarget.getFiducialId();
		inputs.aprilTagDetectionInputs().poseAmbiguity = bestTarget.getPoseAmbiguity();
		inputs.aprilTagDetectionInputs().timestamp = result.getTimestampSeconds();
		inputs.aprilTagDetectionInputs().latency = result.metadata.getLatencyMillis();

		Logger.processInputs(logPath + "/aprilTagDetection", inputs.aprilTagDetectionInputs());
	}

	private static Optional<PhotonPipelineResult> processResults(
		List<PhotonPipelineResult> unreadResults,
		Predicate<PhotonPipelineResult> filter
	) {
		return unreadResults.stream()
			.filter(PhotonPipelineResult::hasTargets) // make sure there are any targets in this frame
			.filter(filter) // filter out results that don't pass the filter
			.sorted(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds)) // sort by timestamp
			.reduce((a, b) -> b); // get the latest result in the stream, if exists
	}


	public void updateInputs() {
		updateHardwareInputs();
		updateAprilTagDetectionInputs();
	}

}
