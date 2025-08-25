package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;

public record DetectedObjectObseration(double timestampSeconds, Translation2d robotRelativeObjectTranslation, DetectedObjectType type) {

	public DetectedObjectObseration() {
		this(0, Translation2d.kZero, null);
	}

}
