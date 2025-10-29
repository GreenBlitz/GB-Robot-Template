package frc.robot.vision.interfaces;

import frc.robot.vision.DetectedObjectObservation;

import java.util.ArrayList;
import java.util.Optional;

public interface ObjectDetector {

	Optional<ArrayList<DetectedObjectObservation>> getRobotRelativeObjectTranslation();

}
