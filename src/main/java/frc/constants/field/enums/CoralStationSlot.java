package frc.constants.field.enums;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.Field;

public enum CoralStationSlot {

	R1(0),
	R2(1),
	R3(2),
	R4(3),
	R5(4),
	R6(5),
	R7(6),
	R8(7),
	R9(8),
	L1(9),
	L2(10),
	L3(11),
	L4(12),
	L5(13),
	L6(14),
	L7(15),
	L8(16),
	L9(17);

	private final int index;
	private final Pose2d position;

	CoralStationSlot(int index) {
		this.index = index;
		this.position = Field.getCoralStationSlotsPose2d(this);
	}

	public int getIndex() {
		return this.index;
	}

	public Pose2d getPosition() {
		return this.position;
	}

}
