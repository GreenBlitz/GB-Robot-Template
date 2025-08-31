package frc.robot.vision;

import java.util.Optional;

public enum DetectedObjectType {

	ALGAE("algae", 0.206375);

	private final String name;
	private final double centerHeightFromFloorMeters;

	DetectedObjectType(String name, double centerHeightFromFloorMeters) {
		this.name = name;
		this.centerHeightFromFloorMeters = centerHeightFromFloorMeters;
	}

	public String getName() {
		return name;
	}

	public double getCenterHeightFromFloorMeters() {
		return centerHeightFromFloorMeters;
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
