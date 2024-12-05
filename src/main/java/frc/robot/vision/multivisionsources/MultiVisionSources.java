package frc.robot.vision.multivisionsources;

import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.VisionSource;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiVisionSources<T extends VisionSource<? extends RawVisionData>> {

	private final List<T> visionSources;

	@SafeVarargs
	public MultiVisionSources(T... visionSources) {
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(List<T> visionSources) {
		this.visionSources = visionSources;
	}

	protected List<T> getVisionSources() {
		return visionSources;
	}

	public List<RawVisionData> getAllAvailablePoseData() {
		List<RawVisionData> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			visionSource.updateEstimation();
			Optional<? extends RawVisionData> rawData = visionSource.getRawVisionEstimation();
			rawData.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

}
