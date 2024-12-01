package frc.robot.poseestimator;

public enum PoseArrayEntryValue {

	X_VALUE(0),
	Y_VALUE(1),
	ROTATION_VALUE(2);

	private final int entryValue;

	PoseArrayEntryValue(int entryValue) {
		this.entryValue = entryValue;
	}

	public int getEntryValue() {
		return entryValue;
	}

	public static final int POSE_ARRAY_LENGTH = 3;

}
