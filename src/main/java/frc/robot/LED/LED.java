package frc.robot.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LED implements ILED, ILogicLED{

	private LED led;

	private Color color;

	private AddressableLED addressableLED;

	private AddressableLEDBuffer addressableLEDBuffer;

	private Timer LEDBlinkTimer;

	@Override
	public void setSingleLEDColor(Color color, int index) {
		addressableLEDBuffer.setLED(index, color);
	}

    @Override
    public void setSectionColor(Color color, int startIndex, int endIndex) {
		for (int i = startIndex; i < endIndex; i++) {
			setSingleLEDColor(color, i);
		}
    }

    @Override
	public void singleLEDTurnOff(int index) {
		setSingleLEDColor(Color.kBlack, index);
	}

	@Override
	public void sectionTurnOff(int startIndex, int endIndex){
		setSectionColor(Color.kBlack, 0, LEDConstatns.LEDStrip.LED_LENGTH);
	}

	public void Blink(int startIndex, int endIndex) {
		if ((LEDBlinkTimer.get()) % (LEDConstatns.LEDStrip.BLINK_DURATION * 2) >= LEDConstatns.LEDStrip.BLINK_DURATION) {
			led.sectionTurnOff(startIndex, endIndex);
		} else {
			led.setSectionColor(color, startIndex, endIndex);
		}
	}
	@Override
	public void LarsonAnimation(int startIndex, int endIndex)) {


	}

}
