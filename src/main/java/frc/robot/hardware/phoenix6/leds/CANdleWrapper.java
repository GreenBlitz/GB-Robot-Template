package frc.robot.hardware.phoenix6.leds;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.Phoenix6Util;

public class CANdleWrapper extends CANdle {

	private final int DEFAULT_CONFIG_NUMBER_OF_TRIES = 1;

	private final int numberOfLeds;

	public CANdleWrapper(int deviceId, int numberOfLeds) {
		this(new Phoenix6DeviceID(deviceId), numberOfLeds);
	}

	public CANdleWrapper(Phoenix6DeviceID ctreDeviceID, int numberOfLeds) {
		super(ctreDeviceID.id(), ctreDeviceID.busChain().getChainName());
		this.numberOfLeds = numberOfLeds;
	}

	public ErrorCode applyConfiguration(CANdleConfiguration configuration, int numberOfTries) {
		return Phoenix6Util.checkWithRetry(() -> this.configAllSettings(configuration), numberOfTries);
	}

	public ErrorCode applyConfiguration(CANdleConfiguration configuration) {
		return applyConfiguration(configuration, DEFAULT_CONFIG_NUMBER_OF_TRIES);
	}

	public ErrorCode setColor(Color color, int startIndex, int amountOfLedsToAffect) {
		return this.setLEDs((int) color.red, (int) color.green, (int) color.green, 0, startIndex, amountOfLedsToAffect);
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

	public ErrorCode animateColorFlowAnimation(Color color, int speed, int amountOfLedsToAffect, int startIndex) {
		return this.animate(
			new ColorFlowAnimation(
				(int) color.red,
				(int) color.green,
				(int) color.green,
				0,
				speed,
				amountOfLedsToAffect,
				ColorFlowAnimation.Direction.Forward,
				startIndex
			)
		);
	}

	public ErrorCode animateColorFlowAnimation(Color color, int speed, int amountOfLedsToAffect) {
		return this.animate(
			new ColorFlowAnimation(
				(int) color.red,
				(int) color.green,
				(int) color.green,
				0,
				speed,
				amountOfLedsToAffect,
				ColorFlowAnimation.Direction.Forward
			)
		);
	}

	public ErrorCode animateColorFlowAnimation(Color color, int speed, double startIndex) {
		return this.animateColorFlowAnimation(color, speed, numberOfLeds - (int) startIndex, (int) startIndex);
	}

	public ErrorCode animateColorFlowAnimation(Color color, int speed) {
		return this.animateColorFlowAnimation(color, speed, numberOfLeds);
	}

	public boolean isConnected() {
		return this.getTemperature() != 0; // we will see if it's not connected and this is the cleanest solution we found
	}

}
