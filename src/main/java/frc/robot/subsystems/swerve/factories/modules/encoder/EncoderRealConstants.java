package frc.robot.subsystems.swerve.factories.modules.encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.constants.GlobalConstants;
import frc.robot.hardware.angleencoder.CANCoderEncoder;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.PhoenixProUtils;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.module.stuffs.EncoderStuff;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

class EncoderRealConstants {

	private static final int APPLY_CONFIG_RETRIES = 10;

	private static CANcoderConfiguration generateEncoderConfig() {
		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

		return encoderConfig;
	}

	protected static EncoderStuff generateEncoderStuff(String logPath, Phoenix6DeviceID encoderDeviceID) {
		CANcoder cancoder = new CANcoder(encoderDeviceID.ID(), encoderDeviceID.busChain().getChainName());
		MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
		cancoder.getConfigurator().refresh(magnetSensorConfigs);
		CANcoderConfiguration caNcoderConfiguration = generateEncoderConfig();
		caNcoderConfiguration.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
		if (!PhoenixProUtils.checkWithRetry(() -> cancoder.getConfigurator().apply(caNcoderConfiguration), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		Phoenix6AngleSignal positionSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(cancoder.getPosition(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);

		return new EncoderStuff(new CANCoderEncoder(logPath, cancoder), positionSignal);
	}

}
