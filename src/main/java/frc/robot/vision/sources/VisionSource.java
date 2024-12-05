package frc.robot.vision.sources;

import frc.robot.vision.rawdata.RawVisionData;

import java.util.Optional;

public interface VisionSource<ReturnType extends RawVisionData> {

	void updateEstimation();

	Optional<ReturnType> getRawVisionEstimation();

}
