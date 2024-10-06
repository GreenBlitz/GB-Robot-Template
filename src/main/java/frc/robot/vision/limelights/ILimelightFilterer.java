package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.List;
import java.util.function.Function;

public interface ILimelightFilterer {

	void setEstimatedPoseAtTimestampFunction(Function<Double, Pose2d> getEstimatedPoseAtTimestamp);

	void updateGyroAngles(GyroAngleValues gyroAngleValues);

	List<VisionObservation> getFilteredVisionObservations();

	List<VisionObservation> getAllAvailableLimelightRawData();

	boolean isPoseEstimationCorrect();

}
