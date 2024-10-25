package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.sources.VisionSource;
import frc.utils.time.TimeUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class MultiVisionSources {

	private final List<VisionSource<RawVisionData>> visionSources;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	@SafeVarargs
	public MultiVisionSources(VisionSource<RawVisionData>... visionSources) {
		this.visionSources = List.of(visionSources);
	}

	public void setEstimatedPoseAtTimestampFunction(Function<Double, Pose2d> getEstimatedPoseAtTimestamp) {
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
	}

	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		visionSources.forEach(source -> source.updateGyroAngles(gyroAngleValues));
	}

	public List<RawVisionData> getAllAvailablePoseData() {
		List<RawVisionData> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			if(getEstimatedPoseAtTimestamp != null) {
				visionSource.updateCurrentEstimatedPose(getEstimatedPoseAtTimestamp.apply(TimeUtils.getCurrentTimeSeconds()));
			}
			visionSource.update();
			Optional<RawVisionData> rawData = visionSource.getAllData();
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
