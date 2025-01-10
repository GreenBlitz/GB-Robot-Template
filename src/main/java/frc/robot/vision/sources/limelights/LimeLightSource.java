package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.robot.poseestimator.helpers.StandardDeviations3D;
import frc.constants.VisionConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.VisionSource;
import frc.utils.AngleUnit;
import frc.utils.Conversions;
import frc.utils.Filter;
import frc.utils.pose.PoseUtils;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;

import static frc.constants.VisionConstants.LATENCY_BOTPOSE_INDEX;

public class LimeLightSource implements VisionSource<AprilTagVisionData> {

	protected final String logPath;
	protected final String name;
	protected final BooleanSupplier shouldDataBeFiltered;
	protected final LimelightPoseEstimatingMethod useGyroForPoseEstimating;

	protected final NetworkTableEntry robotPoseEntryBotPose2;
	protected final NetworkTableEntry robotPoseEntryBotPose1;
	protected final NetworkTableEntry aprilTagIdEntry;
	protected final NetworkTableEntry aprilTagPoseEntry;
	protected final NetworkTableEntry standardDeviations;

	protected double[] aprilTagPoseArray;
	protected double[] robotPoseArray;
	protected double[] standardDeviationsArray;
	protected Filter<AprilTagVisionData> filter;

	protected LimeLightSource(
		String name,
		String parentLogPath,
		Filter<AprilTagVisionData> filter,
		LimelightPoseEstimatingMethod useGyroForPoseEstimating
	) {
		this.logPath = parentLogPath + name + "/";
		this.name = name;
		this.filter = filter;
		this.shouldDataBeFiltered = () -> getVisionData().map(filter::apply).orElse(true);
		this.useGyroForPoseEstimating = useGyroForPoseEstimating;

		this.robotPoseEntryBotPose2 = getLimelightNetworkTableEntry("botpose_orb_wpiblue");
		this.robotPoseEntryBotPose1 = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
		this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
		this.standardDeviations = getLimelightNetworkTableEntry("stddevs");

		AlertManager.addAlert(
			new PeriodicAlert(Alert.AlertType.ERROR, logPath + "DisconnectedAt", () -> getLimelightNetworkTableEntry("tv").getInteger(-1) == -1)
		);

		update();
		log();
	}

	@Override
	public void update() {
		aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		NetworkTableEntry entry = switch (useGyroForPoseEstimating) {
			case BOTPOSE_1 -> robotPoseEntryBotPose1;
			case BOTPOSE_2 -> robotPoseEntryBotPose2;
		};
		robotPoseArray = entry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		standardDeviationsArray = standardDeviations.getDoubleArray(new double[Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT]);

		log();
	}

	protected Optional<Pair<Pose3d, Double>> getUpdatedPose3DEstimation() {
		int id = getAprilTagID();
		if (id == VisionConstants.NO_APRILTAG_ID) {
			return Optional.empty();
		}

		double timestamp = getTimestamp();

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

	protected double getTimestamp() {
		double processingLatencySeconds = Conversions.milliSecondsToSeconds(robotPoseArray[LATENCY_BOTPOSE_INDEX]);
		return TimeUtils.getCurrentTimeSeconds() - processingLatencySeconds;
	}

	public double getPoseValue(Pose3dComponentsValue entryValue) {
		return robotPoseArray[entryValue.getIndex()];
	}

	public double getAprilTagValueInRobotSpace(Pose3dComponentsValue entryValue) {
		return aprilTagPoseArray[entryValue.getIndex()];
	}


	/**
	 * return the aprilTagID
	 *
	 * @return the current april tag ID. In case of dual-target mode or if no apriltag is detected, returns
	 *         {@code VisionConstants.NO_APRILTAG_ID}
	 */
	protected int getAprilTagID() {
		return (int) aprilTagIdEntry.getInteger(VisionConstants.NO_APRILTAG_ID); // a "safe" cast as long as limelight doesn't break APIs
	}

	@Override
	public Optional<AprilTagVisionData> getVisionData() {
		Optional<Pair<Pose3d, Double>> poseEstimation = getUpdatedPose3DEstimation();
		return poseEstimation.map(
			pose3dDoublePair -> new AprilTagVisionData(
				name,
				pose3dDoublePair.getFirst(),
				pose3dDoublePair.getSecond(),
				new StandardDeviations3D(standardDeviationsArray),
				getAprilTagValueInRobotSpace(Pose3dComponentsValue.Z_VALUE),
				getAprilTagValueInRobotSpace(Pose3dComponentsValue.Y_VALUE),
				getAprilTagID()
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
	public Filter<AprilTagVisionData> applyOnFilter(
		BiFunction<Filter<AprilTagVisionData>, Filter<AprilTagVisionData>, Filter<AprilTagVisionData>> applicationFunction,
		Filter<AprilTagVisionData> filterToApplyWith
	) {
		return this.filter = applicationFunction.apply(this.filter, filterToApplyWith);
	}

	protected NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
	}


	public void log() {
		Logger.recordOutput(logPath + "filterResult/", shouldDataBeFiltered.getAsBoolean());
		Logger.recordOutput(logPath + "botPoseDirectOutput", PoseUtils.poseArrayToPose3D(robotPoseArray, AngleUnit.DEGREES));
		getVisionData().ifPresent((visionData) -> {
			Logger.recordOutput(logPath + "unfilteredVision/", visionData.getEstimatedPose());
			Logger.recordOutput(logPath + "unfilteredVisionProjected/", visionData.getEstimatedPose().toPose2d());
			Logger.recordOutput(logPath + "aprilTagHeightMeters", visionData.getAprilTagHeightMeters());
			Logger.recordOutput(logPath + "lastUpdate", visionData.getTimestamp());
			Logger.recordOutput(logPath + "stdDevs", standardDeviationsArray);
		});
	}

}
