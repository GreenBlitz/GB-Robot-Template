package frc.robot.vision.limelights;

public enum LimelightEntryValue {

	X_AXIS(0),
	Y_AXIS(1),
	Z_AXIS(2),
	ROLL_ANGLE(3),
	PITCH_ANGLE(4),
	YAW_ANGLE(5),
	TOTAL_LATENCY(6);

	private final int limelightIndexValue;

	LimelightEntryValue(int limelightIndexValue) {
		this.limelightIndexValue = limelightIndexValue;
	}

	public int getIndex() {
		return this.limelightIndexValue;
	}

}
