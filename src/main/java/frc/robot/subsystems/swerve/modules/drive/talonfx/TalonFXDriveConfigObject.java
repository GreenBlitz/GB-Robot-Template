package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.LogPaths;
import frc.robot.hardware.talonfx.TalonFXSignals;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.TalonFXWrapper;
import org.littletonrobotics.junction.Logger;

class TalonFXDriveConfigObject {

	private final TalonFXWrapper motor;
	private final TalonFXSignals signals;
	private final String logPath;

	protected TalonFXDriveConfigObject(CTREDeviceID motorID, boolean inverted, TalonFXConfiguration configuration, String logPath) {
		this.motor = new TalonFXWrapper(motorID);
		this.signals = new TalonFXSignals(
			motor.getPosition().clone(),
			motor.getVelocity().clone(),
			motor.getAcceleration().clone(),
			motor.getMotorVoltage().clone(),
			motor.getStatorCurrent().clone()
		);
		this.logPath = logPath;

		configMotor(configuration);
		motor.setInverted(inverted);
		optimizeBusAndSignals();
	}


	private void configMotor(TalonFXConfiguration driveConfiguration) {
		if (!PhoenixProUtils.checkWithRetry(() -> motor.applyConfiguration(driveConfiguration), TalonFXDriveConstants.APPLY_CONFIG_RETRIES)) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + logPath + "ConfigurationFailAt", Timer.getFPGATimestamp());
		}
	}

	private void optimizeBusAndSignals() {
		BaseStatusSignal.setUpdateFrequencyForAll(
			PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
			signals.position(),
			signals.velocity(),
			signals.acceleration()
		);
		BaseStatusSignal.setUpdateFrequencyForAll(GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, signals.current(), signals.voltage());

		motor.optimizeBusUtilization();
	}


	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXSignals getSignals() {
		return signals;
	}

}
