package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.RawVisionPoseData;

import java.util.Optional;

public interface RobotPoseEstimatingVisionSource extends VisionSource {

	void updateGyroAngles(GyroAngleValues gyroAngleValues);

	Optional<Rotation2d> getRobotHeading();

	Optional<RawVisionPoseData> getPoseEstimation();

}
