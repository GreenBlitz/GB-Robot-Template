package frc.robot.vision.interfaces;

import frc.robot.vision.DetectedObjectObservation;

import java.util.List;

public interface ObjectDetector {

	List<DetectedObjectObservation> getRobotRelativeObjectTranslations();

}
