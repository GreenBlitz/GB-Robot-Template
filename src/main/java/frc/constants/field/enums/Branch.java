package frc.constants.field.enums;

public enum Branch {

	A(0, ReefSide.A, true),
	B(1, ReefSide.A, false),

	C(2, ReefSide.B, true),
	D(3, ReefSide.B, false),

	E(4, ReefSide.C, false),
	F(5, ReefSide.C, true),

	G(6, ReefSide.D, false),
	H(7, ReefSide.D, true),

	I(8, ReefSide.E, false),
	J(9, ReefSide.E, true),

	K(10, ReefSide.F, true),
	L(11, ReefSide.F, false);

	private final int index;
	private final ReefSide reefSide;
	private final boolean isLeft;

	Branch(int index, ReefSide reefSide, boolean isLeft) {
		this.index = index;
		this.reefSide = reefSide;
		this.isLeft = isLeft;
	}

	public int getIndex() {
		return index;
	}

	public ReefSide getReefSide() {
		return reefSide;
	}

	public boolean isLeft() {
		return isLeft;
	}

	public static Branch getBranchByReefSideAndSide(ReefSide reefSide, boolean isLeft) {
		return switch (reefSide) {
			case A -> isLeft ? A : B;
			case B -> isLeft ? C : D;
			case C -> isLeft ? F : E;
			case D -> isLeft ? H : G;
			case E -> isLeft ? J : I;
			case F -> isLeft ? K : L;
		};
	}

}
