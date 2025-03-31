package frc.robot.subsystems.swerve.factories.modules.encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.RobotConstants;
import frc.constants.MathConstants;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.Phoenix6Util;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;
import frc.utils.alerts.Alert;
import frc.utils.math.AngleUnit;

class CANCoderEncoderBuilder {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static CANcoderConfiguration buildEncoderConfig() {
		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = MathConstants.HALF_CIRCLE.getRotations();

		return encoderConfig;
	}

	static IAngleEncoder buildEncoder(String logPath, Phoenix6DeviceID encoderDeviceID) {
		CANcoder cancoder = new CANcoder(encoderDeviceID.id(), encoderDeviceID.busChain().getChainName());
		MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
		cancoder.getConfigurator().refresh(magnetSensorConfigs);
		CANcoderConfiguration caNcoderConfiguration = buildEncoderConfig();
		caNcoderConfiguration.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
		if (!Phoenix6Util.checkStatusCodeWithRetry(() -> cancoder.getConfigurator().apply(caNcoderConfiguration), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		return new CANCoderEncoder(logPath, cancoder);
	}

	static EncoderSignals buildSignals(CANCoderEncoder encoder) {
		return new EncoderSignals(
			Phoenix6SignalBuilder.build(
				encoder.getDevice().getPosition(),
				RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
				AngleUnit.ROTATIONS,
				BusChain.SWERVE_CANIVORE
			)
		);
	}

}
