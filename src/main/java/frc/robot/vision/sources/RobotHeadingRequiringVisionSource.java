package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.data.AprilTagVisionData;

import java.util.Optional;

public interface RobotHeadingRequiringVisionSource extends VisionSource<AprilTagVisionData> {

	public void useRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating);

	public Optional<Rotation2d> getRobotHeading();

	public void updateGyroAngleValues(GyroAngleValues gyroAngleValues);

}
