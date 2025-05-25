package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.robot.vision.data.LimeLightAprilTagVisionData;
import frc.utils.AngleUnit;
import frc.utils.math.StandardDeviations3D;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.OrientationState3D;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.utils.Conversions;
import frc.utils.Filter;
import frc.utils.pose.PoseUtil;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class LimeLightSource implements IndpendentHeadingVisionSource, RobotHeadingRequiringVisionSource {

	private final String logPath;
	private final String cameraNetworkTablesName;
	private final String sourceName;
	private final LimelightPoseEstimationMethod poseEstimationMethod;

	private final NetworkTableEntry cameraPoseOffsetEntry;
	private final NetworkTableEntry robotPoseEntryMegaTag2;
	private final NetworkTableEntry robotPoseEntryMegaTag1;
	private final NetworkTableEntry aprilTagIdEntry;
	private final NetworkTableEntry aprilTagPoseEntry;
	private final NetworkTableEntry standardDeviations;
	private final NetworkTableEntry hardwareMetricsEntry;
	private final NetworkTableEntry robotOrientationEntry;
	private final NetworkTableEntry computingPipelineLatencyEntry;
	private final NetworkTableEntry captureLatencyEntry;
	private final NetworkTableEntry mutableFramesToSkipEntry;

	private double[] aprilTagPoseArray;
	private double[] robotPoseArray;
	private double[] standardDeviationsArray;
	private double[] hardwareMetricsArray;

	private double computingPipeLineLatency;
	private double captureLatency;
	private int lastSeenAprilTagId;
	private BooleanSupplier shouldDataBeFiltered;
	private Filter<? super AprilTagVisionData> filter;
	private OrientationState3D robotOrientationState;
	private boolean isTemperatureBeingRegulated;

	protected LimeLightSource(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset,
		LimelightPoseEstimationMethod poseEstimationMethod
	) {
		this.logPath = parentLogPath + cameraNetworkTablesName + "/" + sourceName + "/";
		this.cameraNetworkTablesName = cameraNetworkTablesName;
		this.sourceName = sourceName;
		this.shouldDataBeFiltered = () -> getVisionData().map(filter::apply).orElse(true);
		this.filter = filter;
		this.poseEstimationMethod = poseEstimationMethod;

		this.cameraPoseOffsetEntry = getLimelightNetworkTableEntry("camerapose_robotspace_set");
		this.robotPoseEntryMegaTag2 = getLimelightNetworkTableEntry("botpose_orb_wpiblue");
		this.robotPoseEntryMegaTag1 = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_robotspace");
		this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
		this.standardDeviations = getLimelightNetworkTableEntry("stddevs");
		this.hardwareMetricsEntry = getLimelightNetworkTableEntry("hw");
		this.robotOrientationEntry = getLimelightNetworkTableEntry("robot_orientation_set");
		this.computingPipelineLatencyEntry = getLimelightNetworkTableEntry("tl");
		this.captureLatencyEntry = getLimelightNetworkTableEntry("cl");
		this.mutableFramesToSkipEntry = getLimelightNetworkTableEntry("throttle_set");

		this.robotOrientationState = new OrientationState3D();

		AlertManager.addAlert(
			new PeriodicAlert(Alert.AlertType.ERROR, logPath + "DisconnectedAt", () -> getLimelightNetworkTableEntry("tv").getInteger(-1) == -1)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "LimelightTemperatureTooHigh",
				() -> VisionConstants.MAXIMUM_LIMELIGHT_TEMPERATURE_CELSIUS <= getLimeLightTemperature()
			)
		);

		updateCameraPoseOffset(cameraPoseOffset);
		update();
		log();
	}

	@Override
	public void update() {
		lastSeenAprilTagId = getAprilTagID();
		robotOrientationEntry.setDoubleArray(robotOrientationState.asArray());
		aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		NetworkTableEntry entry = switch (poseEstimationMethod) {
			case MEGATAG_1 -> robotPoseEntryMegaTag1;
			case MEGATAG_2 -> robotPoseEntryMegaTag2;
		};
		robotPoseArray = entry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		standardDeviationsArray = standardDeviations.getDoubleArray(new double[Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT]);
		hardwareMetricsArray = hardwareMetricsEntry.getDoubleArray(new double[LimeLightHardwareMetrics.values().length]);
		computingPipeLineLatency = computingPipelineLatencyEntry.getDouble(0D);
		captureLatency = captureLatencyEntry.getDouble(0D);

		log();
	}

	protected double getLatency() {
		return computingPipeLineLatency + captureLatency;
	}

	protected Optional<Pair<Pose3d, Double>> getUpdatedPose3DEstimation() {
		if (lastSeenAprilTagId == VisionConstants.NO_APRILTAG_ID) {
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
		double processingLatencySeconds = Conversions.milliSecondsToSeconds(getLatency());
		return TimeUtil.getCurrentTimeSeconds() - processingLatencySeconds;
	}

	public double getPoseValue(Pose3dComponentsValue entryValue) {
		return robotPoseArray[entryValue.getIndex()];
	}

	public double getAprilTagValueInRobotSpace(Pose3dComponentsValue entryValue) {
		return aprilTagPoseArray[entryValue.getIndex()];
	}

	protected int getAprilTagID() {
		return (int) aprilTagIdEntry.getInteger(VisionConstants.NO_APRILTAG_ID); // a "safe" cast as long as limelight doesn't break APIs
	}

	@Override
	public Optional<AprilTagVisionData> getVisionData() {
		Optional<Pair<Pose3d, Double>> poseEstimation = getUpdatedPose3DEstimation();
		return poseEstimation.map(
			pose3dDoublePair -> new LimeLightAprilTagVisionData(
				sourceName,
				pose3dDoublePair.getFirst(),
				pose3dDoublePair.getSecond(),
				new StandardDeviations3D(
					poseEstimationMethod == LimelightPoseEstimationMethod.MEGATAG_1
						? Arrays.copyOfRange(standardDeviationsArray, 0, Pose3dComponentsValue.values().length)
						: Arrays.copyOfRange(
							standardDeviationsArray,
							Pose3dComponentsValue.values().length,
							2 * Pose3dComponentsValue.values().length
						)
				),
				getAprilTagValueInRobotSpace(Pose3dComponentsValue.Z_VALUE),
				getDistanceFromTag(),
				lastSeenAprilTagId,
				getPoseEstimationMethod()
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
	public void setFilter(Filter<? super AprilTagVisionData> newFilter) {
		this.filter = newFilter;
		this.shouldDataBeFiltered = () -> getVisionData().map(filter::apply).orElse(true);
	}

	@Override
	public Filter<? super AprilTagVisionData> getFilter() {
		return filter;
	}

	protected NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(cameraNetworkTablesName).getEntry(entryName);
	}

	@Override
	public void updateRobotAngleValues(OrientationState3D robotOrientationState) {
		this.robotOrientationState = robotOrientationState;
	}

	public LimelightPoseEstimationMethod getPoseEstimationMethod() {
		return poseEstimationMethod;
	}

	private void updateCameraPoseOffset(Pose3d cameraPoseOffset) {
		this.cameraPoseOffsetEntry.setDoubleArray(PoseUtil.pose3DToPoseArray(cameraPoseOffset, AngleUnit.DEGREES));
	}

	private double getDistanceFromTag() {
		return Math.sqrt(
			Math.pow(getAprilTagValueInRobotSpace(Pose3dComponentsValue.X_VALUE), 2)
				+ Math.pow(getAprilTagValueInRobotSpace(Pose3dComponentsValue.Y_VALUE), 2)
				+ Math.pow(getAprilTagValueInRobotSpace(Pose3dComponentsValue.Z_VALUE), 2)
		);
	}

	public int getFPSCount() {
		return (int) hardwareMetricsArray[LimeLightHardwareMetrics.FPS.getIndex()];
	}

	public int getCPUTemperature() {
		return (int) hardwareMetricsArray[LimeLightHardwareMetrics.CPU_TEMPERATURE.getIndex()];
	}

	public int getRAMUsage() {
		return (int) hardwareMetricsArray[LimeLightHardwareMetrics.RAM_USAGE.getIndex()];
	}

	public int getLimeLightTemperature() {
		return (int) hardwareMetricsArray[LimeLightHardwareMetrics.LIMELIGHT_TEMPERATURE.getIndex()];
	}

	public void setSkippedFramesProcessing(int framesCount) {
		mutableFramesToSkipEntry.setInteger(framesCount);
	}

	public int getSkippedFramesProcessing() {
		return (int) mutableFramesToSkipEntry.getInteger(-1); // fallback value -1 is never accessed unless older Limelight firmware is used
	}

	public void log() {
		Logger.recordOutput(logPath + "filterResult", shouldDataBeFiltered.getAsBoolean());
		Logger.recordOutput(logPath + "temperature", getLimeLightTemperature());
		Logger.recordOutput(logPath + "RAMUsage", getRAMUsage());
		Logger.recordOutput(logPath + "CPUTemperature", getCPUTemperature());
		Logger.recordOutput(logPath + "FPS", getFPSCount());
		getVisionData().ifPresent(visionData -> {
			Logger.recordOutput(logPath + "unfiltered3DVision", visionData.getEstimatedPose());
			Logger.recordOutput(logPath + "stdDevs", standardDeviationsArray);
		});
	}

}
