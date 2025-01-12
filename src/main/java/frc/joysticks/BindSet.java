package frc.joysticks;

public enum BindSet {

	NO_JOYSTICK(0),
	NO_BINDINGS(1),
	SWERVE_TEST(2),
	SWERVE_CALIBRATION(3);

	private final int index;

	BindSet(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
