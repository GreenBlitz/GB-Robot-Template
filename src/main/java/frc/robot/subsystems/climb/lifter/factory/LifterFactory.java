package frc.robot.subsystems.climb.lifter.factory;

import frc.robot.subsystems.climb.lifter.Lifter;

public class LifterFactory {

	public static Lifter create(String logPath) {
		return Falcon500LifterBuilder.createLifter(logPath);
	}

}
