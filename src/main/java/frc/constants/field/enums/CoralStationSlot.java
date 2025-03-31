package frc.constants.field.enums;

public enum CoralStationSlot {

	R1(0, CoralStation.RIGHT),
	R2(1, CoralStation.RIGHT),
	R3(2, CoralStation.RIGHT),
	R4(3, CoralStation.RIGHT),
	R5(4, CoralStation.RIGHT),
	R6(5, CoralStation.RIGHT),
	R7(6, CoralStation.RIGHT),
	R8(7, CoralStation.RIGHT),
	R9(8, CoralStation.RIGHT),
	L1(0, CoralStation.LEFT),
	L2(1, CoralStation.LEFT),
	L3(2, CoralStation.LEFT),
	L4(3, CoralStation.LEFT),
	L5(4, CoralStation.LEFT),
	L6(5, CoralStation.LEFT),
	L7(6, CoralStation.LEFT),
	L8(7, CoralStation.LEFT),
	L9(8, CoralStation.LEFT);

	private final int index;
	private final CoralStation coralStation;

	CoralStationSlot(int index, CoralStation coralStation) {
		this.index = index;
		this.coralStation = coralStation;
	}

	public int getIndex() {
		return this.index;
	}

	public CoralStation getCoralStation() {
		return coralStation;
	}

}
