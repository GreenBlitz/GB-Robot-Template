package frc.robot.vision.cameras.limelight;

import frc.robot.vision.DetectedObjectType;
import frc.utils.alerts.Alert;

import java.util.Optional;


public enum LimelightPipeline {

	APRIL_TAG(0, true),
	OBJECT_DETECTION(1, false, new DetectedObjectType[0]);

	private final int pipelineIndex;
	private final boolean isUsingMT;
	private final DetectedObjectType[] detectedObjectTypes;

	LimelightPipeline(int pipelineIndex, boolean isUsingMT, DetectedObjectType[] detectedObjectTypes) {
		this.pipelineIndex = pipelineIndex;
		this.isUsingMT = isUsingMT;
		this.detectedObjectTypes = detectedObjectTypes;
	}

	LimelightPipeline(int pipelineIndex, boolean isUsingMT) {
		this(pipelineIndex, isUsingMT, new DetectedObjectType[0]);
	}

	public int getPipelineIndex() {
		return pipelineIndex;
	}

	public boolean isDetectingObjects() {
		return detectedObjectTypes.length > 0;
	}

	public boolean isUsingMT() {
		return isUsingMT;
	}

	public Optional<DetectedObjectType> getDetectedObjectType(int index) {
		try {
			return Optional.of(detectedObjectTypes[index]);
		} catch (Exception e) {
			new Alert(Alert.AlertType.WARNING, "ObjectDetections/InvalidObjectID: " + index + "in pipeline " + name()).report();
			return Optional.empty();
		}
	}

}
