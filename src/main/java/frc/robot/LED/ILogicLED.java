package frc.robot.LED;

import com.ctre.phoenix.led.*;

public interface ILogicLED {

	void LarsonAnimation(int startIndex, int endIndex);
	void Blink(int startIndex, int endIndex);


}
