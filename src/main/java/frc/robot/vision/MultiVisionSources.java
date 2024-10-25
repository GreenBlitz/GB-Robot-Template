package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.sources.VisionSource;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiVisionSources {

	private final List<VisionSource<RawVisionData>> visionSources;

	@SafeVarargs
	public MultiVisionSources(VisionSource<RawVisionData>... visionSources) {
		this.visionSources = List.of(visionSources);
	}

	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		visionSources.forEach(source -> source.updateGyroAngles(gyroAngleValues));
	}

	public List<RawVisionData> getAllAvailablePoseData() {
		List<RawVisionData> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
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
