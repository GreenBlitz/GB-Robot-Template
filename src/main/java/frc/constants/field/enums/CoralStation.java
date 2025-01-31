package frc.constants.field.enums;

public enum CoralStation {

	RIGHT(0),
	LEFT(1);

	private final int index;

	CoralStation(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
