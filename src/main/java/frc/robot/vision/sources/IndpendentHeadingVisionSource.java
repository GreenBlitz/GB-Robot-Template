package frc.robot.vision.sources;

import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.HeadingData;
import frc.utils.pose.PoseUtils;

import java.util.Optional;

public interface IndpendentHeadingVisionSource extends VisionSource<AprilTagVisionData> {

	default Optional<HeadingData> getRawHeadingData() {
		return getVisionData().map(PoseUtils::VisionDataToHeadingData);
	};

	default Optional<HeadingData> getFilteredHeadingData() {
		return getFilteredVisionData().map(PoseUtils::VisionDataToHeadingData);
	};

}
