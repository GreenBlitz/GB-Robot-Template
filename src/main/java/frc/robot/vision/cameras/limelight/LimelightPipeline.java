package frc.robot.vision.cameras.limelight;

import frc.robot.vision.DetectedObjectType;


public enum LimelightPipeline {

	APRIL_TAG(0, false, true, new DetectedObjectType[0]),
	OBJECT_DETECTION(1, true, false, new DetectedObjectType[0]);

	private final int pipelineIndex;
	private final boolean isDetectingObjects;
	private final boolean isUsingMT;
	private final int numOfObjects = 0;
	private DetectedObjectType[] detectedObjectTypes = new DetectedObjectType[numOfObjects];

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

	public DetectedObjectType getDetectedObjectType(int index) {
		return detectedObjectTypes[index];
	}

}
