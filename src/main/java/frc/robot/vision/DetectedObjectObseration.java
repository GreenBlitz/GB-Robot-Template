package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;

public record DetectedObjectObseration(DetectedObjectType type, Translation2d robotRelativeObjectTranslation, double timestampSeconds) {

	public DetectedObjectObseration() {
		this(null, Translation2d.kZero, 0);
	}

}
