package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public record DetectedObjectObservation(DetectedObjectType type, Translation3d robotRelativeObjectTranslation,
                                        double timestampSeconds) {

    public DetectedObjectObservation() {
        this(null, Translation3d.kZero, 0);
    }

    public DetectedObjectObservation(DetectedObjectType type, Translation2d Translation2dObject, double timestampSeconds) {
       Translation3d tempTranslation3d = new Translation3d(Translation2dObject.getX(),Translation2dObject.getY(),)

        this(type,tempTranslation3d , timestampSeconds);
    }
}
