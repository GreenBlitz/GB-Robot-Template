package frc.robot.vision.cameras.limelight;

import frc.robot.vision.DetectedObjectType;

import java.util.HashMap;
import java.util.Map;

public enum LimelightPipeline {

	APRIL_TAG(0, false, true),
	OBJECT_DETECTION(1, true, false);

	private final int pipelineIndex;
	private final boolean isDetectingObjects;
	private final boolean isUsingMT;
	private final int numOfObjects = 0;
	private DetectedObjectType[] detectedObjectTypes = new DetectedObjectType[numOfObjects];

	LimelightPipeline(int pipelineIndex, boolean isDetectingObjects, boolean isUsingMT) {


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

	public DetectedObjectType[] getDetectedObjectTypes() {
		DetectedObjectType[] temp = new DetectedObjectType[numOfObjects];
		for (int i = 0; i < detectedObjectTypes.length; i++) {
			temp[i]=detectedObjectTypes[i];
		}
		return temp;
	}
}
