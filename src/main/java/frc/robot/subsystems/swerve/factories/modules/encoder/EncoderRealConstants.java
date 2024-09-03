package frc.robot.subsystems.swerve.factories.modules.encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

class EncoderRealConstants {

	protected static CANcoderConfiguration generateEncoderConfig() {
		CANcoderConfiguration configuration = new CANcoderConfiguration();

		configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

		return configuration;
	}

}
