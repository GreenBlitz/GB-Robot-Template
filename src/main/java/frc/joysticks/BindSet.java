package frc.joysticks;

public enum BindSet {

	NO_JOYSTICK(0),
	NO_BINDINGS(1),
	SWERVE(2),
	SECOND(3);

	private final int index;

	BindSet(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
