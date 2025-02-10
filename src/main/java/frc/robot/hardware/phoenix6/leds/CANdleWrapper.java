package frc.robot.hardware.phoenix6.leds;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.Phoenix6Util;

public class CANdleWrapper extends CANdle {

	private final int DEFAULT_CONFIG_NUMBER_OF_TRIES = 5;

	private final String logPath;
	private final int numberOfLeds;

	public CANdleWrapper(int deviceId, int numberOfLeds, String logPath) {
		this(new Phoenix6DeviceID(deviceId), numberOfLeds, logPath);
	}

	public CANdleWrapper(Phoenix6DeviceID ctreDeviceID, int numberOfLeds, String logPath) {
		super(ctreDeviceID.id(), ctreDeviceID.busChain().getChainName());
		super.clearAnimation(0);
		this.logPath = logPath;
		this.numberOfLeds = numberOfLeds;
	}

	public ErrorCode applyConfiguration(CANdleConfiguration configuration, int numberOfTries) {
		return Phoenix6Util.checkWithRetry(() -> this.configAllSettings(configuration), numberOfTries);
	}

	public ErrorCode applyConfiguration(CANdleConfiguration configuration) {
		return applyConfiguration(configuration, DEFAULT_CONFIG_NUMBER_OF_TRIES);
	}

	public ErrorCode setColor(Color color, int startIndex, int amountOfLedsToAffect) {
		return super.setLEDs((int) color.red, (int) color.green, (int) color.blue, 0, startIndex, amountOfLedsToAffect);
	}

	public ErrorCode setColor(Color color, int startIndex) {
		return this.setColor(color, startIndex, numberOfLeds - startIndex);
	}

	public ErrorCode setColor(Color color, double amountOfLedsToAffect) {
		return this.setColor(color, 0, (int) amountOfLedsToAffect);
	}

	public ErrorCode setColor(Color color) {
		return this.setColor(color, 0);
	}

}
