package frc.robot.poseestimator;

public enum Pose3dComponentsValue {

	X_VALUE(0),
	Y_VALUE(1),
	Z_VALUE(2),
	ROLL_VALUE(3),
	PITCH_VALUE(4),
	YAW_VALUE(5);

	private final int index;

	Pose3dComponentsValue(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

	public static final int POSE3D_COMPONENTS_AMOUNT = values().length;

}
