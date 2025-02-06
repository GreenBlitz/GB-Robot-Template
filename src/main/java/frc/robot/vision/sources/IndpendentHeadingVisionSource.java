package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.TimedValue;
import frc.robot.vision.data.AprilTagVisionData;
import frc.utils.pose.PoseUtil;

import java.util.Optional;

public interface IndpendentHeadingVisionSource extends VisionSource<AprilTagVisionData> {

	default Optional<TimedValue<Rotation2d>> getRawHeadingData() {
		return getVisionData().map(PoseUtil::visionDataToHeadingData);
	};

	default Optional<TimedValue<Rotation2d>> getFilteredHeadingData() {
		return getFilteredVisionData().map(PoseUtil::visionDataToHeadingData);
	};

}
