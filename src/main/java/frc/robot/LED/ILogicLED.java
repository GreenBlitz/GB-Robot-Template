package frc.robot.LED;

public interface ILogicLED {

	void colorFlowAnimation(int startIndex, int endIndex);
	void larsonAnimation(int startIndex, int endIndex, int ledPocketSize);

	void blink(int startIndex, int endIndex);


}
