package frc.constants.field.enums;

public enum Branch {

	A(0, ReefSide.A),
	B(1, ReefSide.A),
	C(2, ReefSide.B),
	D(3, ReefSide.B),
	E(4, ReefSide.C),
	F(5, ReefSide.C),
	G(6, ReefSide.D),
	H(7, ReefSide.D),
	I(8, ReefSide.E),
	J(9, ReefSide.E),
	K(10, ReefSide.F),
	L(11, ReefSide.F);

	private final int index;
	private final ReefSide reefSide;

	Branch(int index, ReefSide reefSide) {
		this.index = index;
		this.reefSide = reefSide;
	}

	public int getIndex() {
		return index;
	}

	public ReefSide getReefSide() {
		return reefSide;
	}

}
