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

	public ArrayList<VisionObservation> getAllAvailablePoseData() {
		ArrayList<VisionObservation> rawPoseData = new ArrayList<VisionObservation>();
		visionSources.forEach(visionSource -> {
			visionSource.updateEstimation();
			Optional<VisionObservation> observation = extractVisionObservation(visionSource.getRawVisionEstimation());
			observation.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

	public ArrayList<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<VisionObservation>();

		for (VisionSource<? extends RawVisionData> visionSource : visionSources) {
			if (!visionSource.shallBeFiltered()) {
				Optional<VisionObservation> observation = extractVisionObservation(visionSource.getRawVisionEstimation());
				observation.ifPresent(estimates::add);
 			}
		}
		return estimates;
	}

	private Optional<VisionObservation> extractVisionObservation(Optional<? extends RawVisionData> optionalRawVisionData) {
		if (optionalRawVisionData.isPresent()) {
			return Optional.of(optionalRawVisionData.get());
		}
 		return Optional.empty();
	}

	private void logObservations(String logPathAddition, List<VisionObservation> observations) {
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(super.getLogPath() + logPathAddition + i, observations.get(i).getEstimatedPose());
		}
	}

	@Override
	protected void subsystemPeriodic() {
		logObservations(VisionConstants.FILTERED_ESTIMATION_LOGPATH_ADDITION, getFilteredVisionObservations());
		logObservations(VisionConstants.NON_FILTERED_ESTIMATION_LOGPATH_ADDITION, getAllAvailablePoseData());
	}
}
