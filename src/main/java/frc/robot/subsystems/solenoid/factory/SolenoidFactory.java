package frc.robot.subsystems.solenoid.factory;

import frc.robot.Robot;
import frc.robot.subsystems.solenoid.SolenoidComponents;

public class SolenoidFactory {

	public static SolenoidComponents create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> SolenoidRealConstants.generateSolenoidComponents(logPath);
			case SIMULATION -> null;
		};
	}

}
