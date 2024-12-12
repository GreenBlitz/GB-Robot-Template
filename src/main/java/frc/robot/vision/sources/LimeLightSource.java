package frc.robot.vision.sources;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.filters.Filter;
import frc.robot.vision.limelights.LimelightEntryValue;
import frc.robot.vision.rawdata.RawAprilTagVisionData;
import frc.utils.Conversions;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtils;

import java.util.Optional;

public class LimeLightSource extends GBSubsystem implements VisionSource<RawAprilTagVisionData> {

	private final NetworkTableEntry robotPoseEntryBotPose2;
	private final NetworkTableEntry RobotPoseEntryBotPose1;
	private final NetworkTableEntry aprilTagIdEntry;
	private final NetworkTableEntry aprilTagPoseEntry;
	private final NetworkTableEntry robotOrientationEntry;
	private final String name;
	private Filter<RawAprilTagVisionData> filter;
	private double[] robotPoseArray;
	private double[] aprilTagPoseArray;
	private Rotation2d robotHeading;
	private LimelightGyroAngleValues gyroAngleValues;
	private boolean useOldRobotPoseEntry;

	public LimeLightSource(String name, String parentLogPath, Filter<RawAprilTagVisionData> filter) {
		super(parentLogPath + name + "/");

		this.name = name;
		this.filter = filter;
		this.robotPoseEntryBotPose2 = getLimelightNetworkTableEntry("botpose_orb_wpiblue");
		this.RobotPoseEntryBotPose1 = getLimelightNetworkTableEntry("botpose_wpiblue");
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
		this.useOldRobotPoseEntry = false;

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				this.getLogPath() + "DisconnectedAt",
				() -> getLimelightNetworkTableEntry("tv").getInteger(-1) == -1
			)
		);
	}

	public void switchToOldBotPose(boolean useOldRobotPose) {
		useOldRobotPoseEntry = useOldRobotPose;
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
		robotPoseArray = (useOldRobotPoseEntry ? RobotPoseEntryBotPose1 : robotPoseEntryBotPose2)
			.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		double[] robotPoseWithoutGyroInput = RobotPoseEntryBotPose1.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
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
	public Optional<RawAprilTagVisionData> getRawVisionData() {
		Optional<Pair<Pose3d, Double>> poseEstimation = getUpdatedPose3DEstimation();
		return poseEstimation.map(
			pose3dDoublePair -> new RawAprilTagVisionData(
				pose3dDoublePair.getFirst(),
				getAprilTagValue(LimelightEntryValue.Y_AXIS),
				getAprilTagValue(LimelightEntryValue.Z_AXIS),
				pose3dDoublePair.getSecond()
			)
		);
	}

	@Override
	public boolean shouldDataBeFiltered() {
		return getRawVisionData().map(filter::doesFilterPasses).orElseGet(() -> true);
	}

	@Override
	public Filter<RawAprilTagVisionData> setFilter(Filter<RawAprilTagVisionData> newFilter) {
		return this.filter = newFilter;
	}

	/**
	 * the robot heading is calculated by the botpose1 algorithm, which does not the current yaw unlike botpose2.
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
