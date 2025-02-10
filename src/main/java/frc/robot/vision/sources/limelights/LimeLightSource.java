package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.utils.math.StandardDeviations3D;
import frc.constants.VisionConstants;
import frc.robot.vision.RobotAngleValues;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.utils.math.AngleUnit;
import frc.utils.Conversions;
import frc.utils.Filter;
import frc.utils.pose.PoseUtil;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

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
	private final NetworkTableEntry robotOrientationEntry;
	private final NetworkTableEntry computingPipelineLatencyEntry;
	private final NetworkTableEntry captureLatencyEntry;

	private double[] aprilTagPoseArray;
	private double[] robotPoseArray;
	private double[] standardDeviationsArray;
	private double computingPipeLineLatency;
	private double captureLatency;
	private int lastSeenAprilTagId;
	private BooleanSupplier shouldDataBeFiltered;
	private Filter<? super AprilTagVisionData> filter;
	private RobotAngleValues robotAngleValues;

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
		this.robotOrientationEntry = getLimelightNetworkTableEntry("robot_orientation_set");
		this.computingPipelineLatencyEntry = getLimelightNetworkTableEntry("tl");
		this.captureLatencyEntry = getLimelightNetworkTableEntry("cl");

		this.robotAngleValues = new RobotAngleValues();
		AlertManager.addAlert(
			new PeriodicAlert(Alert.AlertType.ERROR, logPath + "DisconnectedAt", () -> getLimelightNetworkTableEntry("tv").getInteger(-1) == -1)
		);

		updateCameraPoseOffset(cameraPoseOffset);
		update();
		log();
	}

	@Override
	public void update() {
		lastSeenAprilTagId = getAprilTagID();
		robotOrientationEntry.setDoubleArray(robotAngleValues.asArray());
		Logger.recordOutput(logPath + "gyroAngleValues", robotAngleValues.asArray());
		aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		NetworkTableEntry entry = switch (poseEstimationMethod) {
			case MEGATAG_1 -> robotPoseEntryMegaTag1;
			case MEGATAG_2 -> robotPoseEntryMegaTag2;
		};
		robotPoseArray = entry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		standardDeviationsArray = standardDeviations.getDoubleArray(new double[Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT]);
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
			pose3dDoublePair -> new AprilTagVisionData(
				sourceName,
				pose3dDoublePair.getFirst(),
				pose3dDoublePair.getSecond(),
				new StandardDeviations3D(standardDeviationsArray),
				getAprilTagValueInRobotSpace(Pose3dComponentsValue.Z_VALUE),
				getDistanceFromTag(),
				lastSeenAprilTagId
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
	public void updateRobotAngleValues(RobotAngleValues robotAngleValues) {
		this.robotAngleValues = robotAngleValues;
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

	public void log() {
		Logger.recordOutput(logPath + "filterResult", shouldDataBeFiltered.getAsBoolean());
		Logger.recordOutput(logPath + "megaTagDirectOutput", PoseUtil.toPose3D(robotPoseArray, AngleUnit.DEGREES));
		getVisionData().ifPresent(visionData -> {
			Logger.recordOutput(logPath + "unfiltered3DVision", visionData.getEstimatedPose());
			Logger.recordOutput(logPath + "unfiltered2DVision(Projected)", visionData.getEstimatedPose().toPose2d());
			Logger.recordOutput(logPath + "aprilTagHeightMeters", visionData.getAprilTagHeightMeters());
			Logger.recordOutput(logPath + "lastUpdate", visionData.getTimestamp());
			Logger.recordOutput(logPath + "stdDevs", standardDeviationsArray);
			if (poseEstimationMethod == LimelightPoseEstimationMethod.MEGATAG_1) {
				Logger.recordOutput(logPath + "robotMegaTag1Heading", visionData.getEstimatedPose().getRotation().toRotation2d());
			}
		});
	}

}
