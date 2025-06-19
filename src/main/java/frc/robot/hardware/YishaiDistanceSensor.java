package frc.robot.hardware;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.utils.Conversions;

public class YishaiDistanceSensor {

	private static final double SCALING_SLOPE = 0.0002, SCALING_INTERCEPT_POINT = -200;

	private final DutyCycle sensorDutyCycle;

	public YishaiDistanceSensor(DigitalInput digitalInput) {
		this.sensorDutyCycle = new DutyCycle(digitalInput);
	}

	public double getDistanceMeters() {
		return Conversions.centimetersToMeters(sensorDutyCycle.getHighTimeNanoseconds() * SCALING_SLOPE + SCALING_INTERCEPT_POINT);
	}

}
