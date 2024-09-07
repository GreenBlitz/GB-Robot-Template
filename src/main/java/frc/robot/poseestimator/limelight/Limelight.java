package frc.robot.poseestimator.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.GBSubsystem;

import java.util.Optional;

public class Limelight extends GBSubsystem {

	private NetworkTableEntry robotPoseEntry, idEntry, tagPoseEntry;

	private String name;

	public Limelight(String limelightName) {
		super(VisionConstants.LIMELIGHT_LOGPATH_PREFIX + limelightName + "/");
		this.name = limelightName;
		robotPoseEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue");
		tagPoseEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("targetpose_cameraspace");
		idEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("tid");
	}

	public Optional<Pair<Pose2d, Double>> getUpdatedPose2DEstimation() {
		int id = (int) idEntry.getInteger(-1);
		if (id == -1) {
			return Optional.empty();
		}

		double[] poseArray = robotPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		double processingLatencySeconds = poseArray[LimelightArrayValue.TOTAL_LATENCY.getValue()] / 1000;
		double timestamp = Timer.getFPGATimestamp() - processingLatencySeconds;

		Pose2d robotPose = new Pose2d(
			poseArray[LimelightArrayValue.X_AXIS.getValue()],
			poseArray[LimelightArrayValue.Y_AXIS.getValue()],
			Rotation2d.fromDegrees(poseArray[LimelightArrayValue.PITCH_ANGLE.getValue()])
		);

		return Optional.of(new Pair<>(robotPose, timestamp));
	}

	public double getAprilTagHeight() {
		double[] poseArray = tagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		return poseArray[LimelightArrayValue.Y_AXIS.getValue()];
	}

	public boolean isAprilTagInProperHeight() {
		boolean aprilTagHeightConfidence = Math.abs(getAprilTagHeight() - VisionConstants.APRIL_TAG_HEIGHT_METERS)
			< VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
		return aprilTagHeightConfidence;
	}

	public double getDistanceFromAprilTag() {
		double[] poseArray = tagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		return poseArray[LimelightArrayValue.Z_AXIS.getValue()];
	}

	public boolean hasTarget() {
		return getUpdatedPose2DEstimation().isPresent();
	}

	@Override
	protected void subsystemPeriodic() {}

}
