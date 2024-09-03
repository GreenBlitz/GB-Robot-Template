package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPaths;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.robot.subsystems.swerve.gyro.GyroConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.Pigeon2Wrapper;
import org.littletonrobotics.junction.Logger;

public class Pigeon2ConfigObject {

	private static final int APPLY_CONFIG_RETRIES = 10;


	private final Pigeon2Wrapper gyro;
	private final StatusSignal<Double> yawSignal;

	public Pigeon2ConfigObject(CTREDeviceID gyroID, Pigeon2Configuration configuration, String logPathPrefix) {
		this.gyro = new Pigeon2Wrapper(gyroID);
		this.yawSignal = gyro.getYaw().clone();

		configGyro(configuration, logPathPrefix + GyroConstants.LOG_PATH_ADDITION);
		optimizeBusAndSignals();
	}

	private void configGyro(Pigeon2Configuration configuration, String logPath) {
		if (!PhoenixProUtils.checkWithRetry(() -> gyro.getConfigurator().apply(configuration), APPLY_CONFIG_RETRIES)) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + logPath + "ConfigurationFailAt", Timer.getFPGATimestamp());
		}
	}

	private void optimizeBusAndSignals() {
		BaseStatusSignal.setUpdateFrequencyForAll(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ, yawSignal);
		gyro.optimizeBusUtilization();
	}


	protected Pigeon2Wrapper getGyro() {
		return gyro;
	}

	protected StatusSignal<Double> getYawSignal() {
		return yawSignal;
	}

}
