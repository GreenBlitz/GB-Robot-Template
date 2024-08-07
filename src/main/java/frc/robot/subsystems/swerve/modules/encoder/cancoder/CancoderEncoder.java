package frc.robot.subsystems.swerve.modules.encoder.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.encoder.EncoderInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;

public class CancoderEncoder implements IEncoder {

	private final CANcoder encoder;

	private final StatusSignal<Double> positionSignal, velocitySignal, voltageSignal;

	public CancoderEncoder(CTREDeviceID encoderID, CANcoderConfiguration configuration) {
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
		PhoenixProUtils.checkWithRetry(
			() -> encoder.getConfigurator().apply(encoderConfiguration),
			CancoderEncoderConstants.NUMBER_OF_STATUS_CODE_RETRIES
		);
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
