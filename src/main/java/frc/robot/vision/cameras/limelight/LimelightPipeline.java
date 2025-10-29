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
	private HashMap<Integer,DetectedObjectType> map = new HashMap<>();

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

	public HashMap<Integer, DetectedObjectType> getMap() {
		return map;
	}
}
