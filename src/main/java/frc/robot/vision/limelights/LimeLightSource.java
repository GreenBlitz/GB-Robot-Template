package frc.robot.vision.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.VisionRawData;
import frc.robot.vision.VisionSource;
import frc.utils.Conversions;
import frc.utils.time.TimeUtils;

import java.util.Optional;

public class LimeLightSource extends GBSubsystem implements VisionSource<VisionRawData> {

	private final NetworkTableEntry robotPoseEntry;
	private final NetworkTableEntry aprilTagIdEntry;
	private final NetworkTableEntry aprilTagPoseEntry;
	private final NetworkTableEntry robotOrientationEntry;
	private final NetworkTableEntry robotPoseForHeadingEntry;
	private final String name;
	private double[] robotPoseArray;
	private double[] aprilTagPoseArray;
	private Rotation2d robotHeading;
	private GyroAngleValues gyroAngleValues;

	public LimeLightSource(String name, String logPath) {
		super(logPath + name + "/");

		this.name = name;
		this.robotPoseEntry = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
		this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
		this.robotOrientationEntry = getLimelightNetworkTableEntry("robot_orientation_set");
		this.robotPoseForHeadingEntry = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.gyroAngleValues = new GyroAngleValues(0, 0, 0, 0, 0, 0);
	}

	@Override
	public void update(GyroAngleValues gyroAngleValues) {
		this.gyroAngleValues = gyroAngleValues;
		robotOrientationEntry.setDoubleArray(
			new double[] {
				gyroAngleValues.yaw(),
				gyroAngleValues.yawRate(),
				gyroAngleValues.pitch(),
				gyroAngleValues.pitchRate(),
				gyroAngleValues.roll(),
				gyroAngleValues.rollRate()
			}
		);
		robotPoseArray = robotPoseEntry.getDoubleArray(new double[LimeLightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[LimeLightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		double[] robotPoseWithoutGyroInput = robotPoseForHeadingEntry
			.getDoubleArray(new double[LimeLightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		robotHeading = Rotation2d.fromDegrees(robotPoseWithoutGyroInput[LimelightEntryValue.YAW_ANGLE.getIndex()]);
	}

	@Override
	public Optional<Pair<Pose3d, Double>> getUpdatedPose3DEstimation() {
		int id = (int) aprilTagIdEntry.getInteger(LimeLightConstants.NO_APRILTAG_ID);
		if (id == LimeLightConstants.NO_APRILTAG_ID) {
			return Optional.empty();
		}

		double processingLatencySeconds = Conversions.milliSecondsToSeconds(robotPoseArray[LimelightEntryValue.TOTAL_LATENCY.getIndex()]);
		double timestamp = TimeUtils.getCurrentTimeSeconds() - processingLatencySeconds;

		Pose3d robotPose = new Pose3d(
			getPoseValue(LimelightEntryValue.X_AXIS),
			getPoseValue(LimelightEntryValue.Y_AXIS),
			getPoseValue(LimelightEntryValue.Z_AXIS),
			new Rotation3d(
				Math.toRadians(getPoseValue(LimelightEntryValue.ROLL_ANGLE)),
				Math.toRadians(getPoseValue(LimelightEntryValue.PITCH_ANGLE)),
				Math.toRadians(getPoseValue(LimelightEntryValue.YAW_ANGLE))
			)
		);
		return Optional.of(new Pair<>(robotPose, timestamp));
	}

	public double getPoseValue(LimelightEntryValue entryValue) {
		return robotPoseArray[entryValue.getIndex()];
	}

	public double getAprilTagValue(LimelightEntryValue entryValue) {
		return aprilTagPoseArray[entryValue.getIndex()];
	}

	@Override
	public Optional<VisionRawData> getAllOtherData() {
		Optional<Pair<Pose3d, Double>> poseEstimation = getUpdatedPose3DEstimation();
		return poseEstimation.map(pose3dDoublePair -> new VisionRawData(
			pose3dDoublePair.getFirst(),
			getAprilTagValue(LimelightEntryValue.Y_AXIS),
			getAprilTagValue(LimelightEntryValue.Z_AXIS),
			pose3dDoublePair.getSecond()
		));
	}

	@Override
	public Optional<Rotation2d> getRobotHeading() {
		int id = (int) aprilTagIdEntry.getInteger(LimeLightConstants.NO_APRILTAG_ID);
		if (id == LimeLightConstants.NO_APRILTAG_ID) {
			return Optional.empty();
		}
		return Optional.of(robotHeading);
	}

	private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
	}

}
