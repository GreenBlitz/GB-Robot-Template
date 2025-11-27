package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;

public record DetectedObjectObservation(DetectedObjectType type, Translation2d robotRelativeObjectTranslation, double timestampSeconds) {
-
	public DetectedObjectObservation() {
		this(null, Translation2d.kZero, 0);
	}

}
