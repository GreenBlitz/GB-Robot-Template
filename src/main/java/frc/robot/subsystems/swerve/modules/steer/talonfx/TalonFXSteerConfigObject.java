package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.GlobalConstants;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.PhoenixProUtils;
import frc.robot.hardware.phoenix6.TalonFXWrapper;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.alerts.Alert;

class TalonFXSteerConfigObject {

	private final TalonFXWrapper motor;
	private final TalonFXSteerSignals signals;
	private final String logPath;

	protected TalonFXSteerConfigObject(
		Phoenix6DeviceID motorID,
		boolean inverted,
		int encoderID,
		TalonFXConfiguration configuration,
		String logPath
	) {
		this.motor = new TalonFXWrapper(motorID);
		this.signals = new TalonFXSteerSignals(
			motor.getPosition().clone(),
			motor.getVelocity().clone(),
			motor.getAcceleration().clone(),
			motor.getMotorVoltage().clone()
		);
		this.logPath = logPath;

		configMotor(encoderID, configuration);
		motor.setInverted(inverted);
		optimizeBusAndSignals();
	}

	private void configMotor(int encoderID, TalonFXConfiguration configuration) {
		if (encoderID != TalonFXSteerConstants.NO_ENCODER_ID) {
			configuration.Feedback.FeedbackRemoteSensorID = encoderID;
		}
		if (!PhoenixProUtils.checkWithRetry(() -> motor.applyConfiguration(configuration), TalonFXSteerConstants.APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.WARNING, logPath + "ConfigurationFailAt").report();
		}
	}

	private void optimizeBusAndSignals() {
		BaseStatusSignal.setUpdateFrequencyForAll(
			PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
			signals.positionSignal(),
			signals.velocitySignal(),
			signals.accelerationSignal()
		);
		BaseStatusSignal.setUpdateFrequencyForAll(GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, signals.voltageSignal());

		motor.optimizeBusUtilization();
	}


	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXSteerSignals getSignals() {
		return signals;
	}

}
