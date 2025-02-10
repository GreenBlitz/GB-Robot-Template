package frc.robot.subsystems.solenoid.factory;

import frc.robot.subsystems.solenoid.Solenoid;

public class SolenoidFactory {

	public static Solenoid create(String logPath) {
		return SolenoidSparkMaxBuilder.createSolenoid(logPath);
	}

}
