package frc.robot.vision.interfaces;

import frc.robot.vision.DetectedObjectObseration;

import java.util.Optional;

public interface ObjectDetector {

	Optional<DetectedObjectObseration> getRobotRelativeObjectTranslation();

}
