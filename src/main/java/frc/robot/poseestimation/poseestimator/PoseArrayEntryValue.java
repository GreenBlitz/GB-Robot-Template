package frc.robot.poseestimation.poseestimator;

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

}
