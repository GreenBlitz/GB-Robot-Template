package frc.robot.subsystems.swerve.modules.steer.talonfx;

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

class TalonFXSteerConfigObject {

	private final TalonFXWrapper motor;
	private final TalonFXSignals signals;
	private final String logPath;

	protected TalonFXSteerConfigObject(
		CTREDeviceID motorID,
		boolean inverted,
		int encoderID,
		TalonFXConfiguration configuration,
		String logPath
	) {
		this.motor = new TalonFXWrapper(motorID);
		this.signals = new TalonFXSignals(
			motor.getPosition().clone(),
			motor.getVelocity().clone(),
			motor.getAcceleration().clone(),
			motor.getStatorCurrent().clone(),
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
