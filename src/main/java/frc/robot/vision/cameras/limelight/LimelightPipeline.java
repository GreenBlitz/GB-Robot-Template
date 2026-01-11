package frc.robot.vision.cameras.limelight;

import frc.robot.vision.DetectedObjectType;
import frc.utils.alerts.Alert;

import java.util.Optional;


public enum LimelightPipeline {

	APRIL_TAG(0, true),
	NEURAL_DETECTION(1, false, true, false, new DetectedObjectType[0]),
	COLOR_DETECTION(2, false, false, true, new DetectedObjectType[0]);

	private final int pipelineIndex;
	private final boolean isUsingMT;
	private final boolean isNeuralDetecting;
	private final boolean isColorDetecting;
	private final DetectedObjectType[] detectedObjectTypes;

	LimelightPipeline(
		int pipelineIndex,
		boolean isUsingMT,
		boolean isNeuralDetecting,
		boolean isColorDetecting,
		DetectedObjectType[] detectedObjectTypes
	) {
		this.pipelineIndex = pipelineIndex;
		this.isUsingMT = isUsingMT;
		this.isNeuralDetecting = isNeuralDetecting;
		this.isColorDetecting = isColorDetecting;
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

	public boolean isNeuralDetecting() {
		return isNeuralDetecting;
	}

	public boolean isColorDetecting() {
		return isColorDetecting;
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
