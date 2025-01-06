package frc.joysticks;

public enum BindSet {

	NO_JOYSTICK(0),
	NO_BINDINGS(1),
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
			case 0 -> NO_JOYSTICK;
			case 1 -> NO_BINDINGS;
			case 2 -> SWERVE;
			case 3 -> SECOND;
			case 4 -> TESTING;
			default -> NO_BINDINGS;
		};
	}

}
