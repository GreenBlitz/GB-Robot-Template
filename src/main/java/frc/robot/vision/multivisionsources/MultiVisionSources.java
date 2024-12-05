package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.VisionSource;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class MultiVisionSources<ReturnType extends RawVisionData, CameraType extends VisionSource<ReturnType>> {

	private final List<CameraType> visionSources;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	@SafeVarargs
	public MultiVisionSources(CameraType... visionSources) {
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(List<CameraType> visionSources) {
		this.visionSources = visionSources;
	}

	public void setEstimatedPoseAtTimestampFunction(Function<Double, Pose2d> getEstimatedPoseAtTimestamp) {
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
	}

	protected List<CameraType> getVisionSources() {
		return visionSources;
	}

	public List<ReturnType> getAllAvailablePoseData() {
		List<ReturnType> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			visionSource.updateEstimation();
			Optional<ReturnType> rawData = visionSource.getRawVisionEstimation();
			rawData.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

}
