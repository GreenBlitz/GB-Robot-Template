package frc.robot.vision.multivisionsources;

import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiVisionSources<T extends VisionSource<? extends RawVisionData>> extends GBSubsystem {

	private final List<T> visionSources;

	@SafeVarargs
	public MultiVisionSources(String logPath, T... visionSources) {
		super(logPath);
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(String logPath, List<T> visionSources) {
		super(logPath);
		this.visionSources = visionSources;
	}

	protected List<T> getVisionSources() {
		return visionSources;
	}

	public List<VisionObservation> getAllAvailablePoseData() {
		List<VisionObservation> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			visionSource.updateEstimation();
			Optional<VisionObservation> rawData = visionSource.getRawVisionEstimation();
			rawData.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (VisionSource<? extends RawVisionData> visionSource : visionSources) {
			if (!visionSource.shallBeFiltered()) {
				Optional<VisionObservation> observation = visionSource.getRawVisionEstimation();
				estimates.add()
			}
		}
		return estimates;
	}


	private void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(super.getLogPath() + VisionConstants.FILTERED_ESTIMATION_LOGPATH_ADDITION + i, observations.get(i).robotPose());
		}
	}


}
