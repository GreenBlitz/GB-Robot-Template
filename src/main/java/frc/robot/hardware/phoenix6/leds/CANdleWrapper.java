package frc.robot.hardware.phoenix6.leds;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.Phoenix6Util;

public class CANdleWrapper extends CANdle {

	private static final int DEFAULT_CONFIG_NUMBER_OF_TRIES = 1;

	public CANdleWrapper(int deviceId) {
		this(new Phoenix6DeviceID(deviceId));
	}

	public CANdleWrapper(Phoenix6DeviceID ctreDeviceID) {
		super(ctreDeviceID.id(), ctreDeviceID.busChain().getChainName());
	}

	public ErrorCode applyConfiguration(CANdleConfiguration configuration, int numberOfTries) {
		return Phoenix6Util.checkWithRetry(() -> this.configAllSettings(configuration), numberOfTries);
	}

	public ErrorCode applyConfiguration(CANdleConfiguration configuration) {
		return applyConfiguration(configuration, DEFAULT_CONFIG_NUMBER_OF_TRIES);
	}

	public boolean isConnected() {
		return this.getTemperature() == 0;
	}

}
