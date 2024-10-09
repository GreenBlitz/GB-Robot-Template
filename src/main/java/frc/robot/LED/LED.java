package frc.robot.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LED implements ILED {

	private LED led;

	private Color color;

	private AddressableLED addressableLED;

	private AddressableLEDBuffer addressableLEDBuffer;

	private Timer LEDBlinkTimer;

	@Override
	public void setColor(Color color, int index) {
		this.addressableLEDBuffer.setLED(index, color);
	}

	@Override
	public void turnOff(int index) {
		setColor(new Color(0, 0, 0), index);
	}

	public void Blink() {
		if ((LEDBlinkTimer.get()) % (LEDConstatns.LEDStrip.BLINK_DURATION * 2) >= LEDConstatns.LEDStrip.BLINK_DURATION) {
			led.turnOff(LEDConstatns.LEDStrip.LED_LENGTH);
		} else {
			led.setColor(color, LEDConstatns.LEDStrip.LED_LENGTH);
		}
	}

}
