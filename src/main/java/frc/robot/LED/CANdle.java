package frc.robot.LED;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.Timer;

import java.awt.*;

public class CANdle implements ILED, ILogicLED {

	private com.ctre.phoenix.led.CANdle caNdle;
	private Timer CANdleBlinkTimer;


	public CANdle() {
		this.caNdle = new com.ctre.phoenix.led.CANdle(LEDConstants.CANdle.ID_PORT);
		this.caNdle.configLEDType(com.ctre.phoenix.led.CANdle.LEDStripType.RGB);
		this.CANdleBlinkTimer = new Timer();
		this.CANdleBlinkTimer.restart();
	}

	@Override
	public void setSectionColor(Color color, int startIndex, int endIndex) {
		caNdle.setLEDs((int) color.getRed(), (int) color.getGreen(), (int) color.getBlue(), 1, startIndex, endIndex);
	}

	@Override
	public void setSingleLEDColor(Color color, int index) {
		caNdle.setLEDs(0, 0, 0);
	}

	@Override
	public void singleLEDTurnOff(int index) {
		caNdle.setLEDs(0, 0, 0);
	}

	@Override
	public void update() {

	}

	@Override
	public void sectionTurnOff(int startIndex, int endIndex) {
		caNdle.setLEDs(0, 0, 0, 1, startIndex, endIndex);
	}

	@Override
	public void colorFlowAnimation(int startIndex, int endIndex) {
		int numLED = endIndex - startIndex;
		ColorFlowAnimation colorFlowAnimation = new ColorFlowAnimation(
			36,
			48,
			48,
			36,
			LEDConstants.LEDStrip.COLORFLOW_ANIMATION_SPEED,
			numLED,
			ColorFlowAnimation.Direction.Forward
		);
		caNdle.animate(colorFlowAnimation);
	}

	@Override
	public void larsonAnimation(int startIndex, int endIndex, int ledPocketSize) {
		int numLED = endIndex - startIndex;
		LarsonAnimation larsonAnimation = new LarsonAnimation(
			36,
			48,
			48,
			36,
			LEDConstants.LEDStrip.LARSON_ANIMATION_SPEED,
			numLED,
			LarsonAnimation.BounceMode.Back,
			ledPocketSize,
			startIndex
		);
		caNdle.animate(larsonAnimation);
	}

	@Override
	public void blink(int startIndex, int endIndex) {
		if ((CANdleBlinkTimer.get()) % (LEDConstants.LEDStrip.BLINK_DURATION * 2) >= LEDConstants.CANdle.BLINK_DURATION) {
			caNdle.setLEDs(0, 0, 0, 0, startIndex, endIndex);
		} else {
			caNdle.setLEDs(232, 315, 46, 9, startIndex, endIndex);
		}
	}


}
