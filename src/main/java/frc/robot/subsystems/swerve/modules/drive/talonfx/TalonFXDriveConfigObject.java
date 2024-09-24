package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.GlobalConstants;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.PhoenixProUtils;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.alerts.Alert;

class TalonFXDriveConfigObject {

	private final TalonFXWrapper motor;
	private final TalonFXDriveSignals signals;
	private final String logPath;

	protected TalonFXDriveConfigObject(Phoenix6DeviceID motorID, boolean inverted, TalonFXConfiguration configuration, String logPath) {
		this.motor = new TalonFXWrapper(motorID);
		this.signals = new TalonFXDriveSignals(
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
		if (
			!PhoenixProUtils.checkWithRetry(() -> motor.applyConfiguration(driveConfiguration), TalonFXDriveConstants.APPLY_CONFIG_RETRIES)
				.isOK()
		) {
			new Alert(Alert.AlertType.WARNING, logPath + "ConfigurationFailAt").report();
		}
	}

	//@formatter:off
	private void optimizeBusAndSignals() {
		BaseStatusSignal.setUpdateFrequencyForAll(
			PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
			signals.positionSignal(),
			signals.velocitySignal(),
			signals.accelerationSignal()
		);
		BaseStatusSignal.setUpdateFrequencyForAll(
			GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			signals.voltageSignal(),
			signals.statorCurrentSignal()
		);

		motor.optimizeBusUtilization();
	}
	//@formatter:on


	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXDriveSignals getSignals() {
		return signals;
	}

}
