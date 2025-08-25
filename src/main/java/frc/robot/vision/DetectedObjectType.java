package frc.robot.vision;

public enum DetectedObjectType {
	;

	private final int classIndex;
	private final double heightMeters;

	DetectedObjectType(int classIndex, double height) {
		this.classIndex = classIndex;
		this.heightMeters = height;
	}

	public int getClassIndex() {
		return classIndex;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

	public static DetectedObjectType getByIndex(int classIndex) {
		for (DetectedObjectType detectedObjectType : values()) {
			if (detectedObjectType.classIndex == classIndex) {
				return detectedObjectType;
			}
		}
		return null;
	}

}
