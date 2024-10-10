package frc.robot.LED;

import frc.robot.RobotType;

public class LEDFactory {

	public static ILED create(RobotType robotType) {
		return switch (robotType) {
			case REAL -> LEDConstants.LEDFactory.LED_IS_CANDLE ? new CANdle() : new LEDStrip();

			case SIMULATION -> null;
		};
	}


}
