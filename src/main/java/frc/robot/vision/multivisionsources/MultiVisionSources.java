package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.VisionSource;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class MultiVisionSources<T extends VisionSource> {

	private final List<T> visionSources;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	@SafeVarargs
	public MultiVisionSources(T... visionSources) {
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(List<T> visionSources) {
		this.visionSources = visionSources;
	}

	public void setEstimatedPoseAtTimestampFunction(Function<Double, Pose2d> getEstimatedPoseAtTimestamp) {
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
	}

	protected List<T> getVisionSources() {
		return visionSources;
	}

	public List<RawVisionData> getAllAvailablePoseData() {
		List<RawVisionData> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			visionSource.updateEstimation();
			Optional<RawVisionData> rawData = visionSource.getRawVisionEstimation();
			rawData.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

}
