package frc.robot.vision.sources.limelights.limelight4;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.limelights.LimeLightSource;
import frc.robot.vision.sources.limelights.LimelightPoseEstimationMethod;
import frc.utils.Filter;
import frc.utils.TimedValue;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class LimeLight4 extends LimeLightSource implements IndpendentHeadingVisionSource, RobotHeadingRequiringVisionSource {

	private final NetworkTableEntry mutableIMUDataEntry;
	private final NetworkTableEntry mutableIMUModeEntry;
	private final NetworkTableEntry mutableInternalIMURelianceEntry;

	private LimelightIMUMode limelightImuMode;
	private double[] imuDataArray;

	public LimeLight4(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset,
		LimelightPoseEstimationMethod poseEstimationMethod,
		LimelightIMUMode defaultLimelightIMUMode,
		int defaultSkippedFramesCount,
		double defaultRatioBetweenIMUAndSource,
		boolean regulateTemperature
	) {
		super(cameraNetworkTablesName, parentLogPath, sourceName, filter, cameraPoseOffset, poseEstimationMethod, regulateTemperature);
		this.mutableIMUDataEntry = getLimelightNetworkTableEntry("imu");
		this.mutableIMUModeEntry = getLimelightNetworkTableEntry("imumode_set");
		this.mutableInternalIMURelianceEntry = getLimelightNetworkTableEntry("imuassistalpha_set");
		setIMUMode(defaultLimelightIMUMode);
		setSkippedFramesProcessing(defaultSkippedFramesCount);
		setInternalIMUReliance(defaultRatioBetweenIMUAndSource);
	}

	public void setIMUMode(LimelightIMUMode limelightImuMode) {
		this.limelightImuMode = limelightImuMode;
		mutableIMUModeEntry.setInteger(limelightImuMode.getAPIValue());
	}

	public LimelightIMUMode getIMMode() {
		return limelightImuMode;
	}

	public void setInternalIMUReliance(double ratioBetweenIMUAndSource) {
		mutableInternalIMURelianceEntry.setDouble(ratioBetweenIMUAndSource);
		if (!limelightImuMode.isHeadingRequiring()) {
			AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.WARNING, "GotInteralIMURelianceButDoesntRequireHeading", () -> true));
		}
	}

	public double getInternalIMUReliance() {
		return mutableInternalIMURelianceEntry.getDouble(-1);
	}

	@Override
	public void update() {
		super.update();
		imuDataArray = mutableIMUDataEntry.getDoubleArray(new double[LimeLightIMUData.length]);
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getRawHeadingData() {
		return limelightImuMode.isIndependent()
			? Optional.of(
				new TimedValue<>(Rotation2d.fromRadians(imuDataArray[LimeLightIMUData.ROBOT_YAW.getIndex()]), TimeUtil.getCurrentTimeSeconds())
			)
			: Optional.empty();
	}

	public double getAccelerationX() {
		return imuDataArray[LimeLightIMUData.ACCELERATION_X.getIndex()];
	}

	public double getAccelerationY() {
		return imuDataArray[LimeLightIMUData.ACCELERATION_Y.getIndex()];
	}

	public double getAccelerationZ() {
		return imuDataArray[LimeLightIMUData.ACCELERATION_Z.getIndex()];
	}

	public double getYaw() {
		return imuDataArray[LimeLightIMUData.YAW.getIndex()];
	}

	public double getPitch() {
		return imuDataArray[LimeLightIMUData.PITCH.getIndex()];
	}

	public double getRoll() {
		return imuDataArray[LimeLightIMUData.ROLL.getIndex()];
	}

	public double getGyroX() {
		return imuDataArray[LimeLightIMUData.GYRO_X.getIndex()];
	}

	public double getGyroY() {
		return imuDataArray[LimeLightIMUData.GYRO_Y.getIndex()];
	}

	public double getGyroZ() {
		return imuDataArray[LimeLightIMUData.GYRO_Z.getIndex()];
	}

	@Override
	public void log() {
		super.log();
		Logger.recordOutput(logPath + "IMUMode", limelightImuMode);

		Logger.recordOutput(logPath + "acceleration/x", getAccelerationX());
		Logger.recordOutput(logPath + "acceleration/y", getAccelerationY());
		Logger.recordOutput(logPath + "acceleration/z", getAccelerationZ());

		Logger.recordOutput(logPath + "angles/yaw", getYaw());
		Logger.recordOutput(logPath + "angles/pitch", getPitch());
		Logger.recordOutput(logPath + "angles/roll", getRoll());

		Logger.recordOutput(logPath + "gyro/x", getGyroX());
		Logger.recordOutput(logPath + "gyro/y", getGyroY());
		Logger.recordOutput(logPath + "gyro/z", getGyroZ());
	}

}
