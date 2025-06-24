package frc.robot.vision.objectdetection;

public enum AllObjectsEntryIndexes {

	TARGET_ID(0),
	TX_NO_CROSS(1),
	TY_NO_CROSS(2),
	TARGET_AREA(3),
	CORNER_0_X(4),
	CORNER_0_Y(5),
	CORNER_1_X(6),
	CORNER_1_Y(7),
	CORNER_2_X(8),
	CORNER_2_Y(9),
	CORNER_3_X(10),
	CORNER_3_Y(11);

	private final int index;

	AllObjectsEntryIndexes(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
