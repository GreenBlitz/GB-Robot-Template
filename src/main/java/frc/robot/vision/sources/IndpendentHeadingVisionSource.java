package frc.robot.vision.sources;

import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.HeadingData;

import java.util.Optional;

public interface IndpendentHeadingVisionSource extends VisionSource<AprilTagVisionData> {

	Optional<HeadingData> getRawHeadingData();

	Optional<HeadingData> getFilteredHeadingData();

}
