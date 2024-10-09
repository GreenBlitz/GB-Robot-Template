package frc.robot.LED;

import frc.robot.RobotType;

public class LEDFactory {

	public static ILED create(RobotType robotType) {
		return switch (robotType) {
			case REAL -> LEDConstatns.LEDFactory.LED_IS_CANDLE ? new Candle() : new AddressableLED();

			case SIMULATION -> new LEDSimulation();
		};
	}


}
