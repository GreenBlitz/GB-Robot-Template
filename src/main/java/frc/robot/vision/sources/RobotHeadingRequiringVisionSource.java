package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.data.AprilTagVisionData;

import java.util.Optional;

public interface RobotHeadingRequiringVisionSource extends VisionSource<AprilTagVisionData> {

	void useRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating);

	Optional<Rotation2d> getRobotHeading();

	void updateGyroAngleValues(GyroAngleValues gyroAngleValues);

}
