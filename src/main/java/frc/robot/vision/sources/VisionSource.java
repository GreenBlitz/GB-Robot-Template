package frc.robot.vision.sources;

import frc.robot.vision.rawdata.RawVisionData;

import java.util.Optional;

public interface VisionSource<T extends RawVisionData> {

	void updateEstimation();

	Optional<T> getRawVisionEstimation();

}
