package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.sources.RobotPoseEstimatingVisionSource;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class MultiVisionSources {

	private final List<RobotPoseEstimatingVisionSource> visionSources;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	public MultiVisionSources(RobotPoseEstimatingVisionSource... visionSources) {
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(List<RobotPoseEstimatingVisionSource> visionSources) {
		this.visionSources = visionSources;
	}

	public void setEstimatedPoseAtTimestampFunction(Function<Double, Pose2d> getEstimatedPoseAtTimestamp) {
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
	}

	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		visionSources.forEach(source -> source.updateGyroAngles(gyroAngleValues));
	}

	public List<RawVisionAprilTagData> getAllAvailablePoseData() {
		List<RawVisionAprilTagData> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			visionSource.updateEstimation();
			Optional<RawVisionAprilTagData> rawData = visionSource.getRawVisionData();
			rawData.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

	public List<Rotation2d> getAllRobotHeadingEstimations() {
		List<Rotation2d> headingEstimations = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			Optional<Rotation2d> heading = visionSource.getRobotHeading();
			heading.ifPresent(headingEstimations::add);
		});
		return headingEstimations;
	}

}
