package frc.constants.field.enums;


public enum CoralStationSlot {

	R1(0, false),
	R2(1, false),
	R3(2, false),
	R4(3, false),
	R5(4, false),
	R6(5, false),
	R7(6, false),
	R8(7, false),
	R9(8, false),
	L1(0, true),
	L2(1, true),
	L3(2, true),
	L4(3, true),
	L5(4, true),
	L6(5, true),
	L7(6, true),
	L8(7, true),
	L9(8, true);

	private final int index;
	private final boolean isLeft;

	CoralStationSlot(int index, boolean isLeft) {
		this.index = index;
		this.isLeft = isLeft;
	}

	public int getIndex() {
		return this.index;
	}

	public boolean isLeft() {
		return isLeft;
	}

}
