package frc.constants.field.enums;

public enum ReefSide {

	A(0),
	B(1),
	C(2),
	D(3),
	E(4),
	F(5);

	private final int index;

	ReefSide(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
