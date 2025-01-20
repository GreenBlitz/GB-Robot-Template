package frc.constants.field.enums;

public enum CagePosition {

	FIELD_WALL(0),
	MIDDLE(1),
	FIELD_CENTER(2);

	private final int index;

	CagePosition(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
