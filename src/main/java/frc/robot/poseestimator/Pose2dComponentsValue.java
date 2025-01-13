package frc.robot.poseestimator;

public enum Pose2dComponentsValue {

	X_VALUE(0),
	Y_VALUE(1),
	ROTATION_VALUE(2);

	private final int index;

	Pose2dComponentsValue(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

	public static final int POSE2D_COMPONENTS_AMOUNT = values().length;

}
