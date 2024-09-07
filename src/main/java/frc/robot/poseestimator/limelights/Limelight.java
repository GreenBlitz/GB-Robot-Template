package frc.robot.poseestimator.limelights;

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

	public Limelight(String limelightName, String hardwareLogPath) {
		super(hardwareLogPath + VisionConstants.LIMELIGHT_LOGPATH_PREFIX + limelightName + "/");

		this.name = limelightName;
		this.robotPoseEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue");
		this.tagPoseEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("targetpose_cameraspace");
		this.idEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("tid");
	}

	public Optional<Pair<Pose2d, Double>> getUpdatedPose2DEstimation() {
		int id = (int) idEntry.getInteger(-1);
		if (id == -1) {
			return Optional.empty();
		}

		double[] poseArray = robotPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		double processingLatencySeconds = poseArray[LimelightEntryValue.TOTAL_LATENCY.getIndex()] / 1000;
		double timestamp = Timer.getFPGATimestamp() - processingLatencySeconds;

		Pose2d robotPose = new Pose2d(
			poseArray[LimelightEntryValue.X_AXIS.getIndex()],
			poseArray[LimelightEntryValue.Y_AXIS.getIndex()],
			Rotation2d.fromDegrees(poseArray[LimelightEntryValue.PITCH_ANGLE.getIndex()])
		);

		return Optional.of(new Pair<>(robotPose, timestamp));
	}

	public double getAprilTagHeight() {
		double[] poseArray = tagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		return poseArray[LimelightEntryValue.Y_AXIS.getIndex()];
	}

	public double getDistanceFromAprilTag() {
		double[] poseArray = tagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		return poseArray[LimelightEntryValue.Z_AXIS.getIndex()];
	}

	@Override
	protected void subsystemPeriodic() {}


}
