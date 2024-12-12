package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.IRobotPoseVisionObservation;
import frc.robot.vision.rawdata.RawAprilTagVisionData;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.LimeLightSource;
import frc.robot.vision.sources.LimelightGyroAngleValues;
import frc.robot.vision.sources.VisionSource;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class MultiVisionSourcesWithExtendedLimelightSupport extends MultiVisionSources<VisionSource<RawAprilTagVisionData>> {

	private final List<VisionSource<RawAprilTagVisionData>> visionSources;

	public MultiVisionSourcesWithExtendedLimelightSupport(String logPath, VisionSource<RawAprilTagVisionData>... visionSources) {
		super(logPath, visionSources);
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSourcesWithExtendedLimelightSupport(String logPath, List<VisionSource<RawAprilTagVisionData>> visionSources) {
		super(logPath, visionSources);
		this.visionSources = visionSources;
	}

	public void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSource<RawAprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limelightSource) {
				limelightSource
					.updateGyroAngles(new LimelightGyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	protected <ReturnType> ArrayList<ReturnType> createMappedCopyOfAprilTagSources(
		List<VisionSource<RawAprilTagVisionData>> list,
		Function<Optional<RawAprilTagVisionData>, Optional<ReturnType>> mapping
	) {
		ArrayList<ReturnType> output = new ArrayList<>();
		list.forEach(visionSource -> {
			visionSource.update();
			Optional<ReturnType> observation = mapping.apply(visionSource.getRawVisionData());
			observation.ifPresent(output::add);
		});
		return output;
	}

	public ArrayList<IRobotPoseVisionObservation> getUnfilteredVisionObservation() {
		return createMappedCopyOfAprilTagSources(visionSources, this::convertToOptionalObservation);
	}

	public ArrayList<IRobotPoseVisionObservation> getFilteredVisionObservations() {
		return createMappedCopyOfAprilTagSources(visionSources, (rawVisionData -> {
			if (rawVisionData.isPresent()) {
				if (!rawVisionData.get().getIsDataValid()) {
					return Optional.empty();
				}
				return convertToOptionalObservation(rawVisionData);
			}
			return Optional.empty();
		}));
	}

	/**
	 * Returns the same optional but extract the object out of the Optional since java doesn't support polymorphism of generics inside optional
	 *
	 * @param optionalRawVisionData: the optional to be converted
	 * @return: new instance that has the same data but java is happier with it
	 */
	private Optional<IRobotPoseVisionObservation> convertToOptionalObservation(Optional<? extends RawAprilTagVisionData> optionalRawVisionData) {
		if (optionalRawVisionData.isPresent()) {
			return Optional.of(optionalRawVisionData.get());
		}
		return Optional.empty();
	}

	public ArrayList<Rotation2d> getRawEstimatedAngles() {
		ArrayList<Rotation2d> output = new ArrayList<>();
		for (VisionSource<RawAprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limeLightSource) {
				limeLightSource.getRobotHeading().ifPresent(output::add);
			} else {
				visionSource.getRawVisionData()
					.ifPresent(
						(RawAprilTagVisionData visionData) -> output.add(Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getZ()))
					);
			}
		}
		return output;
	}

}
