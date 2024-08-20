package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPaths;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.IGyro;
import frc.robot.subsystems.swerve.gyro.GyroConstants;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.Pigeon2Wrapper;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

public class Pigeon2Gyro implements IGyro {

	private static final int APPLY_CONFIG_RETRIES = 10;

	private final Pigeon2 gyro;
	private final StatusSignal<Double> yawSignal;
	private final String logPath;
	private final Queue<Double> yawQueue, timestampQueue;

	public Pigeon2Gyro(CTREDeviceID gyroID, Pigeon2Configuration configuration, String logPathPrefix) {
		this.gyro = new Pigeon2Wrapper(gyroID);
		this.yawSignal = gyro.getYaw().clone();
		this.logPath = logPathPrefix + GyroConstants.LOG_PATH_ADDITION;

		// todo - maybe latency
		this.yawQueue = PhoenixOdometryThread6328.getInstance().registerRegularSignal(gyro, yawSignal);
		this.timestampQueue = PhoenixOdometryThread6328.getInstance().getTimestampQueue();

		configGyro(configuration);
		optimizeBusAndSignals();
	}

	private void configGyro(Pigeon2Configuration configuration) {
		if (!PhoenixProUtils.checkWithRetry(() -> gyro.getConfigurator().apply(configuration), APPLY_CONFIG_RETRIES)) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + logPath + "ConfigurationFailAt", Timer.getFPGATimestamp());
		}
	}

	private void optimizeBusAndSignals() {
		BaseStatusSignal.setUpdateFrequencyForAll(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ, yawSignal);
		gyro.optimizeBusUtilization();
	}


	@Override
	public void setYaw(Rotation2d heading) {
		gyro.setYaw(heading.getDegrees());
	}


	@Override
	public void updateInputs(GyroInputsAutoLogged inputs) {
		inputs.isConnected = BaseStatusSignal.refreshAll(yawSignal).isOK();
		inputs.gyroYaw = Rotation2d.fromDegrees(yawSignal.getValue());
		inputs.yawOdometrySamples = yawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
		inputs.timestampOdometrySamples = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
		yawQueue.clear();
		timestampQueue.clear();
	}

}
