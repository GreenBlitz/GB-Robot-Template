package frc.constants.field.enums;

public enum CoralStationPosition {

	LOWER(0),
	UPPER(1);

	private final int index;

	CoralStationPosition(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
