package frc.robot.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public interface VisionSource<VisionData> {

	void update(GyroAngleValues gyroAngleValues);

	Optional<Pair<Pose3d, Double>> getUpdatedPose3DEstimation();

	Optional<VisionData> getAllOtherData();

	Optional<Rotation2d> getRobotHeading();

}
