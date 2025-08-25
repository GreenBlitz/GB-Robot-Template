package frc.robot.vision.cameras.limelight;

public enum LimelightPipeline {

	APRIL_TAG(0, true),
	OBJECT_DETECTION(1, false);

	private final int pipelineIndex;
	private final boolean isUsingMT;

	LimelightPipeline(int pipelineIndex, boolean isUsingMT) {
		this.pipelineIndex = pipelineIndex;
		this.isUsingMT = isUsingMT;
	}

	public int getPipelineIndex() {
		return pipelineIndex;
	}

	public boolean isUsingMT() {
		return isUsingMT;
	}

}
