package frc.robot.vision.sources.limelights;

public enum LimeLightHardwareMetrics {

	LIMELIGHT_TEMPERATURE(0),
	CPU_USAGE(1),
	RAM_USAGE(2),
	FPS(3);

	private final int index;

	LimeLightHardwareMetrics(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
