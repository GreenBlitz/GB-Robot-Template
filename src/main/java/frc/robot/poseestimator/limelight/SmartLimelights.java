package frc.robot.poseestimator.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


public class SmartLimelights extends GBSubsystem {

	private List<Limelight> limelights;
	private LimelightsHardware limelightHardware;

	public SmartLimelights(String[] limelightNames) {
		super(VisionConstants.DEFAULT_CONFIG.Logpath());
		this.limelightHardware = new LimelightsHardware(limelightNames, VisionConstants.DEFAULT_CONFIG.HardwareLogpath());
		this.limelights = limelightHardware.getAllLimelights();
	}

	public List<Optional<Pair<Pose2d, Double>>> getAll2DEstimates() {
		ArrayList<Optional<Pair<Pose2d, Double>>> estimates = new ArrayList<>();

		for (Limelight limelight : limelights) {
			if (limelightValidityFilter(limelight)) {
				estimates.add(limelight.getUpdatedPose2DEstimation());
			}
		}

		return estimates;
	}

	public boolean isLimelightedOutputInTolerance(Limelight limelight) {
		Pose2d limelightPosition;
		Transform2d transformDifference;
		Rotation2d rotationDifference;

		Pose2d currentPoseObservation = NetworkTables...; // placeholder for pubsubs, when it'll be
																						// added.

		limelightPosition = limelight.getUpdatedPose2DEstimation().get().getFirst();
		transformDifference = limelightPosition.minus(currentPoseObservation);
		rotationDifference = limelightPosition.getRotation().minus(currentPoseObservation.getRotation());

		return transformDifference.getTranslation().getNorm() <= VisionConstants.DEFAULT_CONFIG.PositionNormTolerance()
			&& rotationDifference.getDegrees() <= VisionConstants.DEFAULT_CONFIG.rotationTolerance().getDegrees();
	}

	public boolean isAprilTagInProperHeight(Limelight limelight) {
		boolean aprilTagHeightConfidence = Math.abs(limelight.getAprilTagHeight() - VisionConstants.APRIL_TAG_HEIGHT_METERS)
				< VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
		return aprilTagHeightConfidence;
	}

	public boolean hasTarget(Limelight limelight) {
		return limelight.getUpdatedPose2DEstimation().isPresent();
	}

	public boolean limelightValidityFilter(Limelight limelight) {
		if (hasTarget(limelight) && isAprilTagInProperHeight(limelight)) {
			return isLimelightedOutputInTolerance(limelight);
		}

		return false;
	}

	public void recordEstimatedPositions() {
		int i = 1;
		for (Optional<Pair<Pose2d, Double>> estimation : getAll2DEstimates()) {
			if (estimation.isPresent()) {
				Logger.recordOutput(
					super.getLogPath() + VisionConstants.ESTIMATION_LOGPATH_PREFIX + i,
					estimation.get().getFirst()
				);
			}
			i++;
		}
	}

	public double getDynamicStandardDeviations(int limelightId) {
		return limelights.get(limelightId).getDistanceFromAprilTag() / VisionConstants.VISION_TO_STANDARD_DEVIATION;
	}

	public boolean hasTarget(int limelightId) {
		return limelights.get(limelightId).hasTarget();
	}

	public Optional<Pair<Pose2d, Double>> getFirstAvailableTarget() {
		for (Optional<Pair<Pose2d, Double>> estimation : getAll2DEstimates()) {
			if (estimation.isPresent()) {
				return estimation;
			}
		}

		return Optional.empty();
	}

	public boolean isConnected() {
		return limelights.get(0).hasTarget();
	}

	@Override
	protected void subsystemPeriodic() {}

}
