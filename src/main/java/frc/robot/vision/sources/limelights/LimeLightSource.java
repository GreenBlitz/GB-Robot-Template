package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.GyroAngleValues;
import frc.constants.VisionConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.GyroRequiringVisionSource;
import frc.utils.Conversions;
import frc.utils.Filter;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtils;

import java.util.Optional;

public class LimeLightSource implements GyroRequiringVisionSource {

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
	private GyroAngleValues gyroAngleValues;
	private boolean useGyroForPoseEstimating;

	public LimeLightSource(String name, String parentLogPath, Filter<AprilTagVisionData> filter) {
		String logPath = parentLogPath + name + "/";
		this.name = name;
		this.filter = filter;
		this.robotPoseEntryBotPose2 = getLimelightNetworkTableEntry("botpose_orb_wpiblue");
		this.robotPoseEntryBotPose1 = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
		this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
		this.robotOrientationEntry = getLimelightNetworkTableEntry("robot_orientation_set");
		this.gyroAngleValues = new GyroAngleValues(Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0);
		this.useGyroForPoseEstimating = true;

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "DisconnectedAt",
				() -> getLimelightNetworkTableEntry("tv").getInteger(-1) == -1
			)
		);
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
		robotPoseArray = (useGyroForPoseEstimating ? robotPoseEntryBotPose2 : robotPoseEntryBotPose1)
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
				name,
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

	@Override
	public void useGyroForPoseEstimating(boolean useGyroForPoseEstimating) {
		this.useGyroForPoseEstimating = useGyroForPoseEstimating;
	}

	/**
	 * the robot heading is calculated by the botpose1 algorithm, which does not have the current yaw unlike botpose2.
	 *
	 * @return optional of the heading, empty iff apriltags are not visible to the camera.
	 */
	@Override
	public Optional<Rotation2d> getRobotHeading() {
		int id = (int) aprilTagIdEntry.getInteger(VisionConstants.NO_APRILTAG_ID);
		if (id == VisionConstants.NO_APRILTAG_ID) {
			return Optional.empty();
		}
		return Optional.of(robotHeading);
	}

	@Override
	public void updateGyroAngleValues(GyroAngleValues gyroAngleValues) {
		this.gyroAngleValues = gyroAngleValues;
	}

	private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
	}

}
