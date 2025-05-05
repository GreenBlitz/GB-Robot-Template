package frc.robot.hardware.phoenix6.leds;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.Phoenix6Util;
import frc.utils.alerts.Alert;

public class CANdleWrapper extends CANdle {

	private final int DEFAULT_NUMBER_OF_CONFIG_RETRIES = 5;
	private final int FLOAT_TO_RGB_UNIT_INTERVAL = 255;

	private final String logPath;
	private final int numberOfLeds;

	public CANdleWrapper(int deviceId, int numberOfLeds, String logPath) {
		this(new Phoenix6DeviceID(deviceId), numberOfLeds, logPath);
	}

	public CANdleWrapper(Phoenix6DeviceID deviceId, int numberOfLeds, String logPath) {
		super(deviceId.id(), deviceId.busChain().getChainName());
		super.clearAnimation(0);
		this.logPath = logPath;
		this.numberOfLeds = numberOfLeds;
	}

	public void applyConfiguration(CANdleConfiguration configuration, int numberOfTries) {
		if (Phoenix6Util.checkErrorCodeWithRetry(() -> configAllSettings(configuration), numberOfTries) != ErrorCode.OK) {
			new Alert(Alert.AlertType.ERROR, logPath + "/ConfigurationFailed").report();
		}
	}

	public void applyConfiguration(CANdleConfiguration configuration) {
		applyConfiguration(configuration, DEFAULT_NUMBER_OF_CONFIG_RETRIES);
	}

	public ErrorCode setColor(Color color, int brightness, int startIndex, int amountOfLedsToAffect) {
		return super.setLEDs(
			(int) (color.red * FLOAT_TO_RGB_UNIT_INTERVAL),
			(int) (color.green * FLOAT_TO_RGB_UNIT_INTERVAL),
			(int) (color.blue * FLOAT_TO_RGB_UNIT_INTERVAL),
			brightness,
			startIndex,
			amountOfLedsToAffect
		);
	}

	public ErrorCode setColorFromIndex(Color color, int brightness, int startIndex) {
		return setColor(color, brightness, startIndex, numberOfLeds - startIndex);
	}

	public ErrorCode setAmountOfLedsToColor(Color color, int brightness, int amountOfLedsToAffect) {
		return setColor(color, brightness, 0, amountOfLedsToAffect);
	}

	public ErrorCode setColor(Color color, int brightness) {
		return setColorFromIndex(color, brightness, 0);
	}

	public ErrorCode clear(int startIndex, int amountOfLedsToAffect) {
		return setColor(Color.kBlack, 0, startIndex, amountOfLedsToAffect);
	}

	public ErrorCode clearFromIndex(int startIndex) {
		return clear(startIndex, numberOfLeds - startIndex);
	}

	public ErrorCode clearAmountOfLeds(int amountOfLedsToAffect) {
		return clear(0, amountOfLedsToAffect);
	}

	public ErrorCode clear() {
		return clearFromIndex(0);
	}

}
