package frc.constants.field.enums;

import frc.utils.pose.Side;

public enum ReefSide {

	A(0, Side.MIDDLE, false),
	B(1, Side.RIGHT, false),
	C(2, Side.RIGHT, true),
	D(3, Side.MIDDLE, true),
	E(4, Side.LEFT, true),
	F(5, Side.LEFT, false);

	private final int index;
	private final Side side;
	private final boolean isFar;

	ReefSide(int index, Side side, boolean isFar) {
		this.index = index;
		this.side = side;
		this.isFar = isFar;
	}

	public int getIndex() {
		return index;
	}

	public Side getSide() {
		return side;
	}

	public boolean isFar() {
		return isFar;
	}

	public static ReefSide getReefSideBySideAndFar(Side side, boolean isFar) {
		return switch (side) {
			case LEFT -> isFar ? E : F;
			case MIDDLE -> isFar ? D : A;
			case RIGHT -> isFar ? C : B;
		};
	}

}
