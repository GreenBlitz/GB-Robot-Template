package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.data.AprilTagVisionData;

import java.util.Optional;

public interface GyroRequiringVisionSource extends VisionSource<AprilTagVisionData> {

	void setUseGyroForPoseEstimating(boolean useGyroForPoseEstimating);

	Optional<Rotation2d> getRobotHeading();

	void updateGyroAngleValues(GyroAngleValues gyroAngleValues);

}
