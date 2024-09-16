package frc.robot.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LED implements ILED {

	private Color color;

	private AddressableLED addressableLED;

	private AddressableLEDBuffer addressableLEDBuffer;


	@Override
	public void setColor(Color color, int index) {
		this.addressableLEDBuffer.setLED(index, color);
	}

	@Override
	public void turnOff(int index) {
		setColor(new Color(0, 0, 0), index);
	}

}
