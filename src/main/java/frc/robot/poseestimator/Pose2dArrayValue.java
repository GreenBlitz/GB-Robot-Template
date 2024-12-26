package frc.robot.poseestimator;

public enum Pose2dArrayValue {

	X_VALUE(0),
	Y_VALUE(1),
	ROTATION_VALUE(2);

	private final int index;

	Pose2dArrayValue(int entryValue) {
		this.index = entryValue;
	}

	public int getIndex() {
		return index;
	}

	public static final int POSE_ARRAY_LENGTH = 3;

}
