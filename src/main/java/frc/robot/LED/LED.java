package frc.robot.LED;

import edu.wpi.first.wpilibj.Timer;

import java.awt.*;

public class LED implements ILogicLED {

	private Timer LEDBlinkTimer;

	private Timer colorFlowTimer;

	private LEDStrip ledStrip;

	public LED() {
		this.ledStrip = new LEDStrip();
		this.LEDBlinkTimer = new Timer();
		this.colorFlowTimer = new Timer();
		this.LEDBlinkTimer.restart();
	}

	public void setLEDColor(Color color) {
		ledStrip.setSectionColor(color, 1, LEDConstants.LEDStrip.LED_LENGTH);
	}

	@Override
	public void blink(int startIndex, int endIndex) {
		if ((LEDBlinkTimer.get()) % (LEDConstants.LEDStrip.BLINK_DURATION * 2) >= LEDConstants.LEDStrip.BLINK_DURATION) {
			ledStrip.sectionTurnOff(startIndex, endIndex);
		} else {
			ledStrip.setSectionColor(LEDConstants.LEDStrip.BLINK_COLOR, startIndex, endIndex);
		}
	}

	@Override
	public void colorFlowAnimation(int startIndex, int endIndex) {
		colorFlowTimer.restart();
		for (int i = startIndex; i < endIndex; i++) {
			double startTime = colorFlowTimer.get();
			ledStrip.setSingleLEDColor(LEDConstants.LEDStrip.COLORFLOW_ANIMATION_COLOR, i);
			while (colorFlowTimer.get() - startTime <= 0.075) {
			}
		}
	}

	@Override
	public void larsonAnimation(int startIndex, int endIndex, int ledPocketSize) {
		int currentStartIndex = startIndex;
		while (currentStartIndex != endIndex) {
			ledStrip.setSectionColor(LEDConstants.LEDStrip.LARSON_ANIMATION_COLOR, startIndex, ledPocketSize);
			ledStrip.singleLEDTurnOff(currentStartIndex);
			currentStartIndex++;
		}
	}

	public void off(){
		this.ledStrip.sectionTurnOff(0, LEDConstants.LEDStrip.LED_LENGTH);
	}

	public void update(){
		ledStrip.update();
	}


}
