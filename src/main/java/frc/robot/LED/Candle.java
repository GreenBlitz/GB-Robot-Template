package frc.robot.LED;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color;

public class Candle implements ILED, ILogicLED {
	private CANdle caNdle;


	public Candle() {
		this.caNdle = new CANdle(LEDConstatns.Candle.ID_PORT);
		this.caNdle.configLEDType(CANdle.LEDStripType.RGB);
	}

	@Override
	public void setSectionColor(Color color, int startIndex, int endIndex) {
		caNdle.setLEDs((int) color.red, (int) color.green, (int) color.blue, 1, startIndex, endIndex);
	}

	@Override
	public void setSingleLEDColor(Color color, int index) {
		caNdle.setLEDs(0,0,0);
	}

	@Override
	public void singleLEDTurnOff(int index) {
		caNdle.setLEDs(0, 0, 0);
	}

	@Override
	public void sectionTurnOff(int startIndex, int endIndex) {
		caNdle.setLEDs(0, 0, 0, 1, startIndex, endIndex);
	}

	@Override
	public void LarsonAnimation(int startIndex, int endIndex) {
		int numLED = endIndex - startIndex;
		LarsonAnimation larsonAnimation = new LarsonAnimation(36,48,48, 36, 2.5, numLED, LarsonAnimation.BounceMode.Back, 5);
		caNdle.animate(larsonAnimation);
	}



}
