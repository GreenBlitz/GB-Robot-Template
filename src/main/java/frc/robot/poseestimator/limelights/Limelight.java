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

	private NetworkTableEntry robotPoseEntry;
	private NetworkTableEntry aprilTagIdEntry;
	private NetworkTableEntry aprilTagPoseEntry;
	private String name;
	private double[] robotPoseArray;
	private double[] aprilTagPoseArray;

	public Limelight(String name, String hardwareLogPath) {
		super(hardwareLogPath + VisionConstants.LIMELIGHT_LOGPATH_PREFIX + name + "/");

		this.name = name;
		this.robotPoseEntry = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
		this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
	}

	public Optional<Pair<Pose2d, Double>> getUpdatedPose2DEstimation() {
		int id = (int) aprilTagIdEntry.getInteger(-1);
		if (id == -1) {
			return Optional.empty();
		}

		robotPoseArray = robotPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);

		double processingLatencySeconds = robotPoseArray[LimelightEntryValue.TOTAL_LATENCY.getIndex()] / 1000;
		double timestamp = Timer.getFPGATimestamp() - processingLatencySeconds;

		Pose2d robotPose = new Pose2d(
			robotPoseArray[LimelightEntryValue.X_AXIS.getIndex()],
			robotPoseArray[LimelightEntryValue.Y_AXIS.getIndex()],
			Rotation2d.fromDegrees(robotPoseArray[LimelightEntryValue.PITCH_ANGLE.getIndex()])
		);
		return Optional.of(new Pair<>(robotPose, timestamp));
	}

	public double getAprilTagHeight() {
		return aprilTagPoseArray[LimelightEntryValue.Y_AXIS.getIndex()];
	}

	public double getDistanceFromAprilTag() {
		return aprilTagPoseArray[LimelightEntryValue.Z_AXIS.getIndex()];
	}

	private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
	}

	@Override
	protected void subsystemPeriodic() {}

}
