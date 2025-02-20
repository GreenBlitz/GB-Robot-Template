package frc.robot.subsystems.climb.solenoid.factory;

import frc.robot.subsystems.climb.solenoid.Solenoid;

public class SolenoidFactory {

	public static Solenoid create(String logPath) {
		return SparkMaxSolenoidBuilder.createSolenoid(logPath);
	}

}
