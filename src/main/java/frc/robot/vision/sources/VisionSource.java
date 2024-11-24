package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.GyroAngleValues;

import java.util.Optional;

public interface VisionSource<VisionData> {

	void updateGyroAngles(GyroAngleValues gyroAngleValues);

	void update();

	Optional<VisionData> getCalculatedData();

	Optional<Rotation2d> getRobotHeading();

	void updateCurrentEstimatedPose(Pose2d estimatedPose);

}
