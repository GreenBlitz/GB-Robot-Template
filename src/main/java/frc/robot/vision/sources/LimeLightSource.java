package frc.robot.vision.sources;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.limelights.LimelightEntryValue;
import frc.utils.Conversions;
import frc.utils.Filter;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtils;

import java.util.Optional;

public class LimeLightSource extends GBSubsystem implements VisionSource<AprilTagVisionData> {

	private final NetworkTableEntry robotPoseEntryBotPose2;
	private final NetworkTableEntry robotPoseEntryBotPose1;
	private final NetworkTableEntry aprilTagIdEntry;
	private final NetworkTableEntry aprilTagPoseEntry;
	private final NetworkTableEntry robotOrientationEntry;
	private final String name;
	private Filter<AprilTagVisionData> filter;
	private double[] robotPoseArray;
	private double[] aprilTagPoseArray;
	private Rotation2d robotHeading;
	private LimelightGyroAngleValues gyroAngleValues;
	private boolean useBotPose1PoseEntry;

	public LimeLightSource(String name, String parentLogPath, Filter<AprilTagVisionData> filter) {
		super(parentLogPath + name + "/");

		this.name = name;
		this.filter = filter;
		this.robotPoseEntryBotPose2 = getLimelightNetworkTableEntry("botpose_orb_wpiblue");
		this.robotPoseEntryBotPose1 = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
		this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
		this.robotOrientationEntry = getLimelightNetworkTableEntry("robot_orientation_set");
		this.gyroAngleValues = new LimelightGyroAngleValues(
			Rotation2d.fromDegrees(0),
			0,
			Rotation2d.fromDegrees(0),
			0,
			Rotation2d.fromDegrees(0),
			0
		);
		this.useBotPose1PoseEntry = false;

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				this.getLogPath() + "DisconnectedAt",
				() -> getLimelightNetworkTableEntry("tv").getInteger(-1) == -1
			)
		);
	}

	public void changedUsedBotPoseVersion(boolean useBotPose1) {
		useBotPose1PoseEntry = useBotPose1;
	}

	public void updateGyroAngles(LimelightGyroAngleValues gyroAngleValues) {
		this.gyroAngleValues = gyroAngleValues;
	}

	@Override
	public void update() {
		robotOrientationEntry.setDoubleArray(
			new double[] {
				gyroAngleValues.yaw().getDegrees(),
				gyroAngleValues.yawRate(),
				gyroAngleValues.pitch().getDegrees(),
				gyroAngleValues.pitchRate(),
				gyroAngleValues.roll().getDegrees(),
				gyroAngleValues.rollRate()}
		);
		robotPoseArray = (useBotPose1PoseEntry ? robotPoseEntryBotPose1 : robotPoseEntryBotPose2)
			.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		double[] robotPoseWithoutGyroInput = robotPoseEntryBotPose1.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		robotHeading = Rotation2d.fromDegrees(robotPoseWithoutGyroInput[LimelightEntryValue.YAW_ANGLE.getIndex()]);
	}

	private Optional<Pair<Pose3d, Double>> getUpdatedPose3DEstimation() {
		int id = (int) aprilTagIdEntry.getInteger(VisionConstants.NO_APRILTAG_ID);
		if (id == VisionConstants.NO_APRILTAG_ID) {
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
	public Optional<AprilTagVisionData> getVisionData() {
		Optional<Pair<Pose3d, Double>> poseEstimation = getUpdatedPose3DEstimation();
		return poseEstimation.map(
			pose3dDoublePair -> new AprilTagVisionData(
				pose3dDoublePair.getFirst(),
				pose3dDoublePair.getSecond(),
				getAprilTagValue(LimelightEntryValue.Y_AXIS),
				getAprilTagValue(LimelightEntryValue.Z_AXIS),
				(int) aprilTagIdEntry.getInteger(VisionConstants.NO_APRILTAG_ID) // a safe cast as long as limelight doesn't break APIs
			)
		);
	}

	@Override
	public Optional<AprilTagVisionData> getFilteredVisionData() {
		if (shouldDataBeFiltered(getVisionData())) {
			return getVisionData();
		} else {
			return Optional.empty();
		}
	}

	private boolean shouldDataBeFiltered(Optional<AprilTagVisionData> data) {
		return data.map(filter::apply).orElseGet(() -> true);
	}

	@Override
	public Filter<AprilTagVisionData> setFilter(Filter<AprilTagVisionData> newFilter) {
		return this.filter = newFilter;
	}

	/**
	 * the robot heading is calculated by the botpose1 algorithm, which does not have the current yaw unlike botpose2.
	 *
	 * @return optional of the heading, empty iff apriltags are not visible to the camera.
	 */
	public Optional<Rotation2d> getRobotHeading() {
		int id = (int) aprilTagIdEntry.getInteger(VisionConstants.NO_APRILTAG_ID);
		if (id == VisionConstants.NO_APRILTAG_ID) {
			return Optional.empty();
		}
		return Optional.of(robotHeading);
	}

	private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
	}

}
