package frc.robot.poseestimator;

public enum Pose2dComponentsValue {

	X_VALUE(0),
	Y_VALUE(1),
	ROTATION_VALUE(2);

	private final int index;

<<<<<<< HEAD:src/main/java/frc/robot/poseestimator/Pose2dComponentsValue.java
	Pose2dComponentsValue(int entryValue) {
		this.index = entryValue;
||||||| parent of 029024848 (refactor(vision): improve variable naming, code reusing, and use constants without direct import):src/main/java/frc/robot/poseestimator/Pose2dArrayValue.java
	Pose2dArrayValue(int entryValue) {
		this.index = entryValue;
=======
	Pose2dArrayValue(int entryIndex) {
		this.index = entryIndex;
>>>>>>> 029024848 (refactor(vision): improve variable naming, code reusing, and use constants without direct import):src/main/java/frc/robot/poseestimator/Pose2dArrayValue.java
	}

	public int getIndex() {
		return index;
	}

	public static final int POSE2D_COMPONENTS_AMOUNT = 3;

}
