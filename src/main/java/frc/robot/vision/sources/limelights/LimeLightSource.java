package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.robot.vision.GyroAngleValues;
import frc.constants.VisionConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.utils.Conversions;
import frc.utils.Filter;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import static frc.constants.VisionConstants.LATENCY_BOTPOSE_INDEX;

public class LimeLightSource implements RobotHeadingRequiringVisionSource {

	private final String logPath;
	private final String name;
	private Filter<AprilTagVisionData> filter;
	private final BooleanSupplier shouldDataBeFiltered;

	private final NetworkTableEntry robotPoseEntryBotPose2;
	private final NetworkTableEntry robotPoseEntryBotPose1;
	private final NetworkTableEntry aprilTagIdEntry;
	private final NetworkTableEntry aprilTagPoseEntry;
	private final NetworkTableEntry robotOrientationEntry;
	private final NetworkTableEntry standardDeviations;

	private double[] robotPoseArray;
	private double[] aprilTagPoseArray;
	private double[] robotPoseWithoutGyroInput;
	private double[] standardDeviationsArray;
	private Rotation2d robotHeading;
	private GyroAngleValues gyroAngleValues;
	private boolean useGyroForPoseEstimating;

	public LimeLightSource(String name, String parentLogPath, Filter<AprilTagVisionData> filter) {
		this.logPath = parentLogPath + name + "/";
		this.name = name;
		this.filter = filter;
		this.shouldDataBeFiltered = () -> getVisionData().map(filter::apply).orElse(true);

		this.robotPoseEntryBotPose2 = getLimelightNetworkTableEntry("botpose_orb_wpiblue");
		this.robotPoseEntryBotPose1 = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
		this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
		this.robotOrientationEntry = getLimelightNetworkTableEntry("robot_orientation_set");
		this.standardDeviations = getLimelightNetworkTableEntry("stddevs");
		this.gyroAngleValues = new GyroAngleValues(Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0);
		this.useGyroForPoseEstimating = true;

		AlertManager.addAlert(
			new PeriodicAlert(Alert.AlertType.ERROR, logPath + "DisconnectedAt", () -> getLimelightNetworkTableEntry("tv").getInteger(-1) == -1)
		);

		log();
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
		robotPoseWithoutGyroInput = robotPoseEntryBotPose1.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		robotHeading = Rotation2d.fromDegrees(robotPoseWithoutGyroInput[Pose3dComponentsValue.YAW_VALUE.getIndex()]);
		standardDeviationsArray = standardDeviations.getDoubleArray(new double[Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT]);

		log();
	}

	private Optional<Pair<Pose3d, Double>> getUpdatedPose3DEstimation() {
		int id = (int) aprilTagIdEntry.getInteger(VisionConstants.NO_APRILTAG_ID);
		if (id == VisionConstants.NO_APRILTAG_ID) {
			return Optional.empty();
		}

		double processingLatencySeconds = Conversions.milliSecondsToSeconds(robotPoseArray[LATENCY_BOTPOSE_INDEX]);
		double timestamp = TimeUtils.getCurrentTimeSeconds() - processingLatencySeconds;

		Pose3d robotPose = new Pose3d(
			getPoseValue(Pose3dComponentsValue.X_VALUE),
			getPoseValue(Pose3dComponentsValue.Y_VALUE),
			getPoseValue(Pose3dComponentsValue.Z_VALUE),
			new Rotation3d(
				Math.toRadians(getPoseValue(Pose3dComponentsValue.ROLL_VALUE)),
				Math.toRadians(getPoseValue(Pose3dComponentsValue.PITCH_VALUE)),
				Math.toRadians(getPoseValue(Pose3dComponentsValue.YAW_VALUE))
			)
		);
		return Optional.of(new Pair<>(robotPose, timestamp));
	}

	public double getPoseValue(Pose3dComponentsValue entryValue) {
		return robotPoseArray[entryValue.getIndex()];
	}

	public double getAprilTagValueInRobotSpace(Pose3dComponentsValue entryValue) {
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
				standardDeviationsArray,
				getAprilTagValueInRobotSpace(Pose3dComponentsValue.Z_VALUE),
				getAprilTagValueInRobotSpace(Pose3dComponentsValue.Y_VALUE),
				(int) aprilTagIdEntry.getInteger(VisionConstants.NO_APRILTAG_ID) // a safe cast as long as limelight doesn't break APIs
			)
		);
	}

	@Override
	public Optional<AprilTagVisionData> getFilteredVisionData() {
		if (shouldDataBeFiltered.getAsBoolean()) {
			return getVisionData();
		} else {
			return Optional.empty();
		}
	}

	@Override
	public Filter<AprilTagVisionData> setFilter(Filter<AprilTagVisionData> newFilter) {
		return this.filter = newFilter;
	}

	@Override
	public void useRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating) {
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


	public void log() {
		Logger.recordOutput(logPath + "filterResult/", shouldDataBeFiltered.getAsBoolean());
		Logger.recordOutput(logPath + "botPose1Output", robotPoseWithoutGyroInput);
		getRobotHeading().ifPresent((heading) -> Logger.recordOutput(logPath + "robotBotPose1Heading", heading));
		getVisionData().ifPresent((visionData) -> {
			Logger.recordOutput(logPath + "unfilteredVision/", visionData.getEstimatedPose());
			Logger.recordOutput(logPath + "unfilteredVisionProjected/", visionData.getEstimatedPose().toPose2d());
			Logger.recordOutput(logPath + "aprilTagHeightMeters", visionData.getAprilTagHeightMeters());
			Logger.recordOutput(logPath + "lastUpdate", visionData.getTimestamp());
			Logger.recordOutput(logPath + "stdDevs", standardDeviationsArray);
		});
	}

}