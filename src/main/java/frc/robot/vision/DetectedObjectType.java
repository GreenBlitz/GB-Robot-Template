package frc.robot.vision;

import java.util.Optional;

public enum DetectedObjectType {
	;

	private final int classIndex;
	private final double heightMeters;

	DetectedObjectType(int classIndex, double heightMeters) {
		this.classIndex = classIndex;
		this.heightMeters = heightMeters;
	}

	public int getClassIndex() {
		return classIndex;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

	public static Optional<DetectedObjectType> getByIndex(int classIndex) {
		for (DetectedObjectType detectedObjectType : values()) {
			if (detectedObjectType.classIndex == classIndex) {
				return Optional.of(detectedObjectType);
			}
		}
		return Optional.empty();
	}

}
