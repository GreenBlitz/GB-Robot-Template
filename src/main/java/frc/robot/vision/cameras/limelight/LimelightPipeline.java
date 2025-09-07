package frc.robot.vision.cameras.limelight;

public enum LimelightPipeline {

	APRIL_TAG(0, false, true),
	OBJECT_DETECTION(1, true, false);

	private final int pipelineIndex;
	private final boolean isDetectingObjects;
	private final boolean isUsingMT;

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

}
