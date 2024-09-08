package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.Field;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;


public class LimelightsFiltered extends GBSubsystem {

	private LimelightRawData limelightHardware;

	public LimelightsFiltered(String[] limelightNames) {
		super(VisionConstants.DEFAULT_CONFIG.logPath());

		this.limelightHardware = new LimelightRawData(limelightNames, VisionConstants.DEFAULT_CONFIG.hardwareLogPath());
	}

	public List<VisionObservation> getFiltered2DEstimates() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightData limelightData : limelightHardware.getAllAvlilableLimelightData()) {
			if (!filterOutLimelight(limelightData)) {
				double standardDeviation = getDynamicStandardDeviations(limelightData);
				double[] standardDeviations = new double[] {standardDeviation};

				estimates
					.add(new VisionObservation(limelightData.EstimatedPosition(), standardDeviations, limelightData.timeStamp()));
			}
		}

		return estimates;
	}

	public boolean isLimelightedOutputInTolerance(LimelightData limelightData) {
		Pose2d limelightPosition;
		Transform2d transformDifference;
		Rotation2d rotationDifference;

		// TODO: placeholder for pubsubs, when it'll be added.
		// ik we shouldn't have todos but it's notable enough when it throws compile errors
		Pose2d currentPoseObservation = NetworkTables...;

		limelightPosition = limelightData.EstimatedPosition();
		transformDifference = limelightPosition.minus(currentPoseObservation);
		rotationDifference = limelightPosition.getRotation().minus(currentPoseObservation.getRotation());

		return transformDifference.getTranslation().getNorm() <= VisionConstants.DEFAULT_CONFIG.positionNormTolerance()
			&& rotationDifference.getDegrees() <= VisionConstants.DEFAULT_CONFIG.rotationTolerance().getDegrees();
	}

	public boolean isAprilTagInProperHeight(LimelightData limelightData) {
		boolean aprilTagHeightConfidence = Math.abs(limelightData.AprilTagHeight() - Field.APRIL_TAG_HEIGHT_METERS)
			< VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
		return aprilTagHeightConfidence;
	}

	public boolean filterOutLimelight(LimelightData limelightData) {
		return !(isAprilTagInProperHeight(limelightData) && isLimelightedOutputInTolerance(limelightData));
	}

	public void recordEstimatedPositions() {
		VisionObservation estimation;
		int logpathSuffix;
		ListIterator<VisionObservation> iterator = getFiltered2DEstimates().listIterator();

		while (iterator.hasNext()) {
			estimation = iterator.next();
			logpathSuffix = iterator.nextIndex();


			Logger.recordOutput(
				super.getLogPath() + VisionConstants.ESTIMATION_LOGPATH_PREFIX + logpathSuffix,
				estimation.visionPose()
			);
		}
	}

	public double getDynamicStandardDeviations(LimelightData limelightData) {
		return limelightData.DistanceFromAprilTag() / VisionConstants.APRIL_TAG_DiSTANCE_TO_STANDARD_DEVIATIONS_FACTOR;
	}

	public VisionObservation getFirstAvailableTarget() {
		return getFiltered2DEstimates().get(0);
	}

	@Override
	protected void subsystemPeriodic() {}


}
