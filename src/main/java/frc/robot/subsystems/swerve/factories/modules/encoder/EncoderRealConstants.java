package frc.robot.subsystems.swerve.factories.modules.encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.constants.IDs;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.PhoenixProUtils;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

class EncoderRealConstants {

	private static final int APPLY_CONFIG_RETRIES = 10;

	protected static CANcoderConfiguration generateEncoderConfig() {
		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

		return encoderConfig;
	}

	public static class EncoderCreator {
		private final CANcoder cancoder;
		private final Phoenix6AngleSignal positionSignal;

		public EncoderCreator(Phoenix6DeviceID encoderDeviceID) {
			this.cancoder = new CANcoder(encoderDeviceID.ID(), encoderDeviceID.busChain().getChainName());
			this.positionSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(cancoder.getPosition(), 50, AngleUnit.ROTATIONS);
		}

		public CANcoder getCANcoderWithConfig(String logPath) {
			MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
			cancoder.getConfigurator().refresh(magnetSensorConfigs);
			CANcoderConfiguration caNcoderConfiguration = generateEncoderConfig();
			caNcoderConfiguration.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
			if (!PhoenixProUtils.checkWithRetry(() -> cancoder.getConfigurator().apply(caNcoderConfiguration), APPLY_CONFIG_RETRIES).isOK()) {
				new Alert(Alert.AlertType.WARNING, logPath + "ConfigurationFailAt").report();
			}
			return cancoder;
		}

		public Phoenix6AngleSignal getPositionSignal() {
			return positionSignal;
		}

	}

	protected static final EncoderCreator FRONT_LEFT = new EncoderCreator(IDs.CANCodersIDs.FRONT_LEFT_ENCODER);
	protected static final EncoderCreator FRONT_RIGHT = new EncoderCreator(IDs.CANCodersIDs.FRONT_RIGHT_ENCODER);
	protected static final EncoderCreator BACK_LEFT = new EncoderCreator(IDs.CANCodersIDs.BACK_LEFT_ENCODER);
	protected static final EncoderCreator BACK_RIGHT = new EncoderCreator(IDs.CANCodersIDs.BACK_RIGHT_ENCODER);


}
