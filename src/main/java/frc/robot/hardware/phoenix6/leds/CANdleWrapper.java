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

	public void animateColorFlowAnimation(Color color, double speed){
		this.animate(new ColorFlowAnimation((int) color.red, (int) color.green, (int) color.green, 0, speed, numberOfLeds, ColorFlowAnimation.Direction.Forward));
	}

	public boolean isConnected() {
		return this.getTemperature() == 0; // we will see if it's not connected and this is the cleanest solution we founded
	}

}
