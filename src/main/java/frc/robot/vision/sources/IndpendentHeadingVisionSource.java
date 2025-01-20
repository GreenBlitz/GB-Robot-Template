package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.vision.data.AprilTagVisionData;
import frc.utils.pose.PoseUtils;

import java.util.Optional;

public interface IndpendentHeadingVisionSource extends VisionSource<AprilTagVisionData> {

	default Optional<TimedValue<Rotation2d>> getRawHeadingData() {
		return getVisionData().map(PoseUtils::visionDataToHeadingData);
	};

	default Optional<TimedValue<Rotation2d>> getFilteredHeadingData() {
		return getFilteredVisionData().map(PoseUtils::visionDataToHeadingData);
	};

}
