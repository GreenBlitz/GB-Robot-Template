package frc.joysticks;

public enum BindSet {

	NONE(0),
	EMPTY(1),
	SWERVE(2),
	SECOND(3),
	TESTING(4);

	private final int index;

	BindSet(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

	public static BindSet getBindSetByIndex(int index) {
		return switch (index) {
			case 0 -> NONE;
			case 1 -> EMPTY;
			case 2 -> SWERVE;
			case 3 -> SECOND;
			case 4 -> TESTING;
			default -> EMPTY;
		};
	}

}
