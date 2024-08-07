package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.TalonFXWrapper;

class TalonFXDriveConfigObject {

	private final TalonFXWrapper motor;
	private final TalonFXDriveSignals signals;

	protected TalonFXDriveConfigObject(CTREDeviceID motorID, boolean inverted, TalonFXConfiguration configuration) {
		this.motor = new TalonFXWrapper(motorID);
		this.signals = new TalonFXDriveSignals(
			motor.getPosition().clone(),
			motor.getVelocity().clone(),
			motor.getAcceleration().clone(),
			motor.getMotorVoltage().clone(),
			motor.getStatorCurrent().clone()
		);

		configMotor(configuration);
		motor.setInverted(inverted);
		optimizeBusAndSignals();
	}


	private void configMotor(TalonFXConfiguration driveConfiguration) {
		PhoenixProUtils.checkWithRetry(
			() -> motor.applyConfiguration(driveConfiguration),
			TalonFXDriveConstants.NUMBER_OF_STATUS_CODE_RETRIES
		);
	}

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


	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXDriveSignals getSignals() {
		return signals;
	}

}
