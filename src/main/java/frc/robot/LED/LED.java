package frc.robot.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LED implements ILED, ILogicLED{

	private LED led;

	private AddressableLED addressableLED;

	private AddressableLEDBuffer addressableLEDBuffer;

	private Timer LEDBlinkTimer;

	private LED() {
		this.addressableLED = new AddressableLED(LEDConstants.LEDStrip.LED_PORT);
		this.addressableLEDBuffer = new AddressableLEDBuffer(LEDConstants.LEDStrip.LED_LENGTH);
		this.addressableLED.setLength(LEDConstants.LEDStrip.LED_LENGTH);
		this.addressableLED.start();
	}

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
		setSectionColor(Color.kBlack, 0, LEDConstants.LEDStrip.LED_LENGTH);
	}

	@Override
	public void Blink(int startIndex, int endIndex) {
		if ((LEDBlinkTimer.get()) % (LEDConstants.LEDStrip.BLINK_DURATION * 2) >= LEDConstants.LEDStrip.BLINK_DURATION) {
			led.sectionTurnOff(startIndex, endIndex);
		} else {
			led.setSectionColor(LEDConstants.LEDStrip.BLINK_COLOR, startIndex, endIndex);
		}
	}
	@Override
	public void LarsonAnimation(int startIndex, int endIndex) {
		for ()

	}

}
