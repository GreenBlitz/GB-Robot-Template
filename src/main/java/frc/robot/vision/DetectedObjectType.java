package frc.robot.vision;

import java.util.Optional;

public enum DetectedObjectType {

	ALGAE(0, 0.41275);

	private final String name;
	private final double heightMeters;

	DetectedObjectType(String name, double heightMeters) {
		this.name = name;
		this.heightMeters = heightMeters;
	}

	public String getName() {
		return name;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

	public static Optional<DetectedObjectType> getByName(String name) {
		for (DetectedObjectType detectedObjectType : values()) {
			if (detectedObjectType.name.equals(name)) {
				return Optional.of(detectedObjectType);
			}
		}
		return Optional.empty();
	}

}
