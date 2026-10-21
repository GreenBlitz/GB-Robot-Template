package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstantsFactory {

	private LEDConstantsFactory() {}

	public static LEDConstants createConstants() {
		return new LEDConstants(4, 50, 0.5, 10, 0.05, Color.kBlack, 255, 255);
	}

}
