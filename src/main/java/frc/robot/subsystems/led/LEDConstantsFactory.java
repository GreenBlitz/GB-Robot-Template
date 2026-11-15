package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class LEDConstantsFactory {

	private LEDConstantsFactory() {}

	public static LEDConstants createConstants() {
		return new LEDConstants(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/LED",
			4,
			50,
			0.5,
			10,
			MetersPerSecond.of(0.05),
			Color.kBlack,
			255,
			255
		);
	}

}
