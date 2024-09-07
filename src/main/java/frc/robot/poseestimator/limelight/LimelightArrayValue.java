package frc.robot.poseestimator.limelight;

public enum LimelightArrayValue {

	X_AXIS(0),
	Y_AXIS(1),
	Z_AXIS(2),
	ROLL_ANGLE(3),
	PITCH_ANGLE(4),
	YAW_ANGLE(5),
	TOTAL_LATENCY(6);


	private int limelightIndexValue;

	private LimelightArrayValue(int limelightIndexValue) {
		this.limelightIndexValue = limelightIndexValue;
	}

	public int getValue() {
		return this.limelightIndexValue;
	}


}
