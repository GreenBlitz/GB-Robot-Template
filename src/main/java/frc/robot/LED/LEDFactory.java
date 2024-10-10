package frc.robot.LED;

import frc.robot.Robot;

public class LEDFactory {

	public static ILED create() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> LEDConstants.LEDFactory.LED_IS_CANDLE ? new CANdle() : new LEDStrip();
			case SIMULATION -> null;
		};
	}


}
