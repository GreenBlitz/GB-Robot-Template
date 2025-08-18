package frc.robot.newvision.cameras.limelight;

public enum LimelightPipeline {

	APRIL_TAG(0);

	private final int pipelineIndex;

	LimelightPipeline(int pipelineIndex) {
		this.pipelineIndex = pipelineIndex;
	}

	public int getPipelineIndex() {
		return pipelineIndex;
	}

}
