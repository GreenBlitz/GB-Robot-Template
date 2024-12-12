package frc.robot.vision.multivisionsources;

import frc.robot.poseestimator.observations.IRobotPoseVisionObservation;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.LimeLightSource;
import frc.robot.vision.sources.LimelightGyroAngleValues;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiVisionSources<VisionSourceType extends VisionSource<? extends RawVisionData>> extends GBSubsystem {

	private final List<VisionSourceType> visionSources;

	@SafeVarargs
	public MultiVisionSources(String logPath, VisionSourceType... visionSources) {
		super(logPath);
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(String logPath, List<VisionSourceType> visionSources) {
		super(logPath);
		this.visionSources = visionSources;
	}

	protected List<VisionSourceType> getVisionSources() {
		return visionSources;
	}

	public ArrayList<IRobotPoseVisionObservation> getUnfilteredVisionObservation() {
		ArrayList<IRobotPoseVisionObservation> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			visionSource.update();
			Optional<IRobotPoseVisionObservation> observation = convertToOptionalObservation(visionSource.getRawVisionData());
			observation.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

	public ArrayList<IRobotPoseVisionObservation> getFilteredVisionObservations() {
		ArrayList<IRobotPoseVisionObservation> estimates = new ArrayList<>();

		for (VisionSource<? extends RawVisionData> visionSource : visionSources) {
			if (!visionSource.shouldDataBeFiltered()) {
				Optional<IRobotPoseVisionObservation> observation = convertToOptionalObservation(visionSource.getRawVisionData());
				observation.ifPresent(estimates::add);
			}
		}
		return estimates;
	}

	/**
	 * Returns the same optional but extract the object out of the Optional since java doesn't support polymorphism of generics inside optional
	 *
	 * @param optionalRawVisionData: the optional to be converted
	 * @return: new instance that has the same data but java is happier with it
	 */
	private Optional<IRobotPoseVisionObservation> convertToOptionalObservation(Optional<? extends RawVisionData> optionalRawVisionData) {
		if (optionalRawVisionData.isPresent()) {
			return Optional.of(optionalRawVisionData.get());
		}
		return Optional.empty();
	}

	private static void logRobotPose(String logPath, String logPathAddition, List<IRobotPoseVisionObservation> observations) {
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(logPath + logPathAddition + i, observations.get(i).getEstimatedPose());
		}
	}

	private void log() {
		logRobotPose(getLogPath(), VisionConstants.FILTERED_DATA_LOGPATH_ADDITION, getFilteredVisionObservations());
		logRobotPose(getLogPath(), VisionConstants.NON_FILTERED_DATA_LOGPATH_ADDITION, getUnfilteredVisionObservation());
	}

	private void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSourceType visionSource : visionSources) {
			if (visionSource instanceof LimeLightSource limelightSource) {
				limelightSource
					.updateGyroAngles(new LimelightGyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	public ArrayList<Rotation2d> getRawEstimatedAngles() {
		ArrayList<Rotation2d> output = new ArrayList<>();
		for (VisionSourceType visionSource : visionSources) {
			if (visionSource instanceof LimeLightSource limeLightSource) {
				limeLightSource.getRobotHeading().ifPresent(output::add);
			} else {
				visionSource.getRawVisionData()
					.ifPresent(
						(RawVisionData visionData) -> output.add(Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getZ()))
					);
			}
		}
		return output;
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
