package frc.robot.hardware.phoenix6.leds;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;
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

	public ErrorCode setColor(int red, int green, int blue, int white, int startIndex, int amountOfLedsToAffect) {
		return ledStrip.setLEDs(red, green, blue, white, startIndex, amountOfLedsToAffect);
	}

	public ErrorCode setColor(int red, int green, int blue, int startIndex, int amountOfLedsToAffect) {
		return ledStrip.setLEDs(red, green, blue, 0, startIndex, amountOfLedsToAffect);
	}

	public ErrorCode setColor(int red, int green, int blue) {
		return ledStrip.setLEDs(red, green, blue);
	}

	public ErrorCode setColor(Color color, int startIndex, int amountOfLedsToAffect) {
		return ledStrip.setColor(color, startIndex, amountOfLedsToAffect);
	}

	public ErrorCode setColor(Color color, int startIndex) {
		return ledStrip.setColor(color, startIndex);
	}

	public ErrorCode setColor(Color color, double amountOfLedsToAffect) {
		return ledStrip.setColor(color, amountOfLedsToAffect);
	}

	public ErrorCode setColor(Color color) {
		return ledStrip.setColor(color);
	}

	public ErrorCode animate(Animation animation) {
		return ledStrip.animate(animation);
	}

	public ErrorCode animate(Animation animation, int animationSlot) {
		return ledStrip.animate(animation, animationSlot);
	}

	public ErrorCode clearAnimation(int animationSlot) {
		return ledStrip.clearAnimation(animationSlot);
	}

	public boolean isConnected() {
		return ledStrip.isConnected();
	}

}
