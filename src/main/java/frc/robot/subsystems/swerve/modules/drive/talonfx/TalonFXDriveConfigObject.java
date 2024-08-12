package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.TalonFXWrapper;
import org.littletonrobotics.junction.Logger;

class TalonFXDriveConfigObject {

	private final TalonFXWrapper motor;
	private final TalonFXDriveSignals signals;
	private final String logPath;

	protected TalonFXDriveConfigObject(CTREDeviceID motorID, boolean inverted, TalonFXConfiguration configuration, String logPath) {
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
		if (!PhoenixProUtils.checkWithRetry(() -> motor.applyConfiguration(driveConfiguration), TalonFXDriveConstants.APPLY_CONFIG_RETRIES)) {
			Logger.recordOutput(logPath + "ConfigurationFailAt", Timer.getFPGATimestamp());
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
