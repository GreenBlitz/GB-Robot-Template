package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.LogPaths;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.TalonFXWrapper;
import org.littletonrobotics.junction.Logger;

class TalonFXSteerConfigObject {

	private final TalonFXWrapper motor;
	private final TalonFXSteerSignals signals;
	private final String logPath;

	protected TalonFXSteerConfigObject(
		CTREDeviceID motorID,
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
		if (!PhoenixProUtils.checkWithRetry(() -> motor.applyConfiguration(configuration), TalonFXSteerConstants.APPLY_CONFIG_RETRIES)) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + logPath + "ConfigurationFailAt", Timer.getFPGATimestamp());
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
