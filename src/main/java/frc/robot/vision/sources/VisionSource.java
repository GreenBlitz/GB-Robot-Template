package frc.robot.vision.sources;

import frc.robot.vision.RawVisionAprilTagData;

import java.util.Optional;

public interface VisionSource<T> {

	void updateEstimation();

	Optional<RawVisionAprilTagData> getRawVisionEstimation();

}
