package frc.robot.newvision.interfaces;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

public interface ObjectDetector {

	public Optional<Translation2d> getRobotRelativeObjectTranslation();

}
