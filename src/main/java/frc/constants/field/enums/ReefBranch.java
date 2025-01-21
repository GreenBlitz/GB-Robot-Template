package frc.constants.field.enums;

public enum ReefBranch {

	A(0),
	B(1),
	C(2),
	D(3),
	E(4),
	F(5),
	G(6),
	H(7),
	I(8),
	J(9),
	K(10),
	L(11);

	private final int index;

	ReefBranch(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
