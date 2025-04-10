package frc.robot.vision.sources.limelights.limelight4;

public enum LimeLightIMUData {

	ROBOT_YAW(0),
	ROLL(1),
	PITCH(2),
	YAW(3),
	GYRO_X(4),
	GYRO_Y(5),
	GYRO_Z(6),
	ACCELERATION_X(7),
	ACCELERATION_Y(8),
	ACCELERATION_Z(9);

	private final int index;
	public final static int length = values().length;

	LimeLightIMUData(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
