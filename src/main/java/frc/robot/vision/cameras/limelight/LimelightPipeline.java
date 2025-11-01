package frc.robot.vision.cameras.limelight;

import frc.robot.vision.DetectedObjectType;
import frc.utils.alerts.Alert;

import java.util.Optional;


public enum LimelightPipeline {

	APRIL_TAG(0, false, true, new DetectedObjectType[0]),
	OBJECT_DETECTION(1, true, false, new DetectedObjectType[0]);

	private final int pipelineIndex;
	private final boolean isDetectingObjects;
	private final boolean isUsingMT;
	private DetectedObjectType[] detectedObjectTypes;

	LimelightPipeline(int pipelineIndex, boolean isDetectingObjects, boolean isUsingMT, DetectedObjectType[] detectedObjectTypes) {
		this.detectedObjectTypes = detectedObjectTypes;
		this.pipelineIndex = pipelineIndex;
		this.isDetectingObjects = isDetectingObjects;
		this.isUsingMT = isUsingMT;
	}

	public int getPipelineIndex() {
		return pipelineIndex;
	}

	public boolean isDetectingObjects() {
		return isDetectingObjects;
	}

	public boolean isUsingMT() {
		return isUsingMT;
	}

	public Optional<DetectedObjectType> getDetectedObjectType(int index) {
		try {
			return Optional.of(detectedObjectTypes[index]);
		} catch (NullPointerException | ArrayIndexOutOfBoundsException e) {
			new Alert(Alert.AlertType.WARNING, "ObjectDetections/InvalidObjectID: ").report();
			return Optional.empty();
		}
	}

}
