package frc.robot.subsystems.swerve.modules.encoder.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.LogPaths;
import frc.robot.poseestimator.PoseEstimatorConstants;
import frc.robot.subsystems.swerve.modules.encoder.EncoderConstants;
import frc.robot.subsystems.swerve.modules.encoder.EncoderInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;
import org.littletonrobotics.junction.Logger;

public class CancoderEncoder implements IEncoder {

	private static final int APPLY_CONFIG_RETRIES = 10;

	private final CANcoder encoder;
	private final StatusSignal<Double> positionSignal, velocitySignal, voltageSignal;
	private final String logPath;

	public CancoderEncoder(CTREDeviceID encoderID, CANcoderConfiguration configuration, String logPathPrefix) {
		this.encoder = new CANcoder(encoderID.ID(), encoderID.busChain().getChainName());
		this.positionSignal = encoder.getPosition().clone();
		this.velocitySignal = encoder.getVelocity().clone();
		this.voltageSignal = encoder.getSupplyVoltage().clone();
		this.logPath = logPathPrefix + EncoderConstants.LOG_PATH_ADDITION;

		configEncoder(configuration);
		optimizeBusAndSignals();
	}

	private void configEncoder(CANcoderConfiguration encoderConfiguration) {
		MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
		encoder.getConfigurator().refresh(magnetSensorConfigs);
		encoderConfiguration.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
		if (!PhoenixProUtils.checkWithRetry(() -> encoder.getConfigurator().apply(encoderConfiguration), APPLY_CONFIG_RETRIES)) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + logPath + "ConfigurationFailAt", Timer.getFPGATimestamp());
		}
	}

	private void optimizeBusAndSignals() {
		BaseStatusSignal.setUpdateFrequencyForAll(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ, positionSignal);
		BaseStatusSignal.setUpdateFrequencyForAll(GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, velocitySignal, voltageSignal);

		encoder.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(EncoderInputsAutoLogged inputs) {
		inputs.isConnected = BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal).isOK();
		inputs.angle = Rotation2d.fromRotations(positionSignal.getValue());
		inputs.velocity = Rotation2d.fromRotations(velocitySignal.getValue());
		inputs.voltage = voltageSignal.getValue();
	}

}
