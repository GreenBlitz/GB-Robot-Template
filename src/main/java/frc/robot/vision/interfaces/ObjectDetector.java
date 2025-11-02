package frc.robot.vision.interfaces;

import frc.robot.vision.DetectedObjectObservation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public interface ObjectDetector {

	Optional<List<DetectedObjectObservation>> getRobotRelativeObjectTranslations();

}
