package frc.robot.subsystems.swerve.modules.encoder.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.encoder.EncoderInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;
import org.littletonrobotics.junction.Logger;

public class CancoderEncoder implements IEncoder {

	private final String logPath;
	private final CANcoder encoder;
	private final StatusSignal<Double> positionSignal, velocitySignal, voltageSignal;

	public CancoderEncoder(String logPathPrefix, CTREDeviceID encoderID, CANcoderConfiguration configuration) {
		this.logPath = logPathPrefix + CancoderEncoderConstants.LOG_PATH_ADDITION;
		this.encoder = new CANcoder(encoderID.ID(), encoderID.busChain().getChainName());
		this.positionSignal = encoder.getPosition().clone();
		this.velocitySignal = encoder.getVelocity().clone();
		this.voltageSignal = encoder.getSupplyVoltage().clone();

		configEncoder(configuration);
		optimizeBusAndSignals();
	}

	private void configEncoder(CANcoderConfiguration encoderConfiguration) {
		MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
		encoder.getConfigurator().refresh(magnetSensorConfigs);
		encoderConfiguration.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
		if (!PhoenixProUtils.checkWithRetry(() -> encoder.getConfigurator().apply(encoderConfiguration), CancoderEncoderConstants.APPLY_CONFIG_RETRIES)) {
			Logger.recordOutput(logPath + "ConfigurationFailAt", Timer.getFPGATimestamp());
		}
	}

	private void optimizeBusAndSignals() {
		BaseStatusSignal.setUpdateFrequencyForAll(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ, positionSignal);
		BaseStatusSignal.setUpdateFrequencyForAll(GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, velocitySignal, voltageSignal);

		encoder.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(ModuleInputsContainer inputs) {
		EncoderInputsAutoLogged encoderInputs = inputs.getEncoderInputs();
		encoderInputs.isConnected = BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal).isOK();
		encoderInputs.angle = Rotation2d.fromRotations(positionSignal.getValue());
		encoderInputs.velocity = Rotation2d.fromRotations(velocitySignal.getValue());
		encoderInputs.voltage = voltageSignal.getValue();
	}

}
