package frc.robot.vision.sources.limelights;

public enum LimeLightHardwareMetrics {

	FPS(0),
	CPU_TEMPERATURE(1),
	RAM_USAGE(2),
	LIMELIGHT_TEMPERATURE(3);

	private final int index;
	public final static int length = values().length;

	LimeLightHardwareMetrics(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
