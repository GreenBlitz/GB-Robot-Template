package frc.constants.field.enums;

public enum CoralStationPosition {

	RIGHT(0),
	LEFT(1);

	private final int index;

	CoralStationPosition(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
