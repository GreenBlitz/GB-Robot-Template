package frc.robot.hardware.phoenix6.leds;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.alerts.Alert;


public class CANdle {

	private final CANdleWrapper ledStrip;
	private final String logPath;

	public CANdle(Phoenix6DeviceID deviceID, int numberOfLeds) {
		ledStrip = new CANdleWrapper(deviceID, numberOfLeds);
		this.logPath = deviceID.busChain().getChainName();
	}

	public String getLogPath() {
		return logPath;
	}

	public void applyConfiguration(CANdleConfiguration config) {
		if (ledStrip.applyConfiguration(config) != ErrorCode.OK) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailed").report();
		}
	}

	public CANdleWrapper getDevice() {
		return ledStrip;
	}

	public ErrorCode getLastError() {
		return ledStrip.getLastError();
	}

	public ErrorCode setAnimation(Animation animation) {
		return ledStrip.animate(animation);
	}

	public ErrorCode setAnimation(Animation animation, int animationSlot) {
		return ledStrip.animate(animation, animationSlot);
	}

	public ErrorCode setLEDs(int red, int green, int blue, int white, int startIndex, int amountOfLEDToApply) {
		return ledStrip.setLEDs(red, green, blue, white, startIndex, amountOfLEDToApply);
	}

	public ErrorCode setLEDs(int red, int green, int blue, int startIndex, int amountOfLEDToApply) {
		return ledStrip.setLEDs(red, green, blue, 0, startIndex, amountOfLEDToApply);
	}

	public ErrorCode setLEDs(int red, int green, int blue) {
		return ledStrip.setLEDs(red, green, blue);
	}

	public ErrorCode clearAnimation(int animationSlot) {
		return ledStrip.clearAnimation(animationSlot);
	}

	public boolean isConnected() {
		return ledStrip.isConnected();
	}

}
