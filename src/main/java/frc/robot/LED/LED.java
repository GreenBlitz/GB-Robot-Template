package frc.robot.LED;

import java.awt.*;

public class LED implements ILED {

	private static CANdle CANDle;

	@Override
	public void setColor(Color color) {
		CANDle.setColor(color);
	}

	@Override
	public void turnOff() {
		CANDle.turnOff();
	}

}
