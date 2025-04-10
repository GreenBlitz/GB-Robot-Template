package frc.robot.vision.sources.limelights.limelight4;

public enum LimelightIMUMode {

	USE_EXTERNAL(0),
	USE_EXTERNAL_ASSIST_INTERNAL(1),
	USE_INTERNAL(2),
	USE_INTERNAL_ASSIST_MEGATAG_1(3),
	USE_INTERNAL_ASSIST_EXTERNAL(4);

	private final int apiValue;

	private LimelightIMUMode(int apiValue) {
		this.apiValue = apiValue;
	}

	public int getAPIValue() {
		return apiValue;
	}

	public boolean isIndependent() {
		return this == USE_INTERNAL || this == USE_INTERNAL_ASSIST_MEGATAG_1 || this == USE_INTERNAL_ASSIST_EXTERNAL;
	}

	public boolean isHeadingRequiring() {
		return this == USE_EXTERNAL_ASSIST_INTERNAL || this == USE_EXTERNAL || this == USE_INTERNAL_ASSIST_EXTERNAL;
	}

}
