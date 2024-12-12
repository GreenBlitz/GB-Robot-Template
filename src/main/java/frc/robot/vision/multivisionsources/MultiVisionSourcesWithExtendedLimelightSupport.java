package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.IRobotPoseVisionObservation;
import frc.robot.vision.rawdata.IRawVisionData;
import frc.robot.vision.rawdata.RawAprilTagVisionData;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.LimeLightSource;
import frc.robot.vision.sources.LimelightGyroAngleValues;
import frc.robot.vision.sources.VisionSource;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiVisionSourcesWithExtendedLimelightSupport extends MultiVisionSources<VisionSource> {

	public MultiVisionSourcesWithExtendedLimelightSupport(String logPath, VisionSource... visionSources) {
		super(logPath, visionSources);
	}

	public MultiVisionSourcesWithExtendedLimelightSupport(String logPath, List<VisionSource> visionSources) {
		super(logPath, visionSources);
	}

	public void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSource visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limelightSource) {
				limelightSource
					.updateGyroAngles(new LimelightGyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	public ArrayList<IRobotPoseVisionObservation> getUnfilteredVisionObservation() {
		return createMappedCopyOfSources(getVisionSources(), this::convertToOptionalObservation);
	}

	public ArrayList<IRobotPoseVisionObservation> getFilteredVisionObservations() {
		return createMappedCopyOfSources(getVisionSources(), (rawVisionData -> {
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
		for (VisionSource visionSource : getVisionSources()) {
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
