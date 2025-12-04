package frc.robot.vision.cameras.limelight;

import frc.robot.vision.DetectedObjectType;
import frc.utils.alerts.Alert;

import java.util.Optional;


public enum LimelightPipeline {

	APRIL_TAG(0, true),
	OBJECT_DETECTION(1, false, true, false, new DetectedObjectType[0]),
	COLOR_DETECTION(2, false, false, true, new DetectedObjectType[0]);

	private final int pipelineIndex;
	private final boolean isUsingMT;
	private final boolean isDetectingObjects;
	private final boolean isDetectingColors;
	private final DetectedObjectType[] detectedObjectTypes;

	LimelightPipeline(
		int pipelineIndex,
		boolean isUsingMT,
		boolean isDetectingObjects,
		boolean isDetectingColors,
		DetectedObjectType[] detectedObjectTypes
	) {
		this.pipelineIndex = pipelineIndex;
		this.isUsingMT = isUsingMT;
		this.isDetectingObjects = isDetectingObjects;
		this.isDetectingColors = isDetectingColors;
		this.detectedObjectTypes = detectedObjectTypes;
	}

	LimelightPipeline(int pipelineIndex, boolean isUsingMT) {
		this(pipelineIndex, isUsingMT, false, false, new DetectedObjectType[0]);
	}

	public int getPipelineIndex() {
		return pipelineIndex;
	}

	public boolean isUsingMT() {
		return isUsingMT;
	}

	public boolean isDetectingObjects() {
		return isDetectingObjects;
	}

	public boolean isDetectingColors() {
		return isDetectingColors;
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
