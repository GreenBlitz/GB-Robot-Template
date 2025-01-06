package frc.robot.vision.sources;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.data.AprilTagVisionData;

import java.util.Optional;

public interface RobotHeadingRequiringVisionSource extends VisionSource<AprilTagVisionData> {

	void useRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating);

	Optional<Pair<Rotation2d, Double>> getRobotHeading();

	void updateGyroAngleValues(GyroAngleValues gyroAngleValues);

}
