package frc.robot.subsystems.lifter.factory;

import frc.robot.subsystems.lifter.Lifter;

public class LifterFactory {

	public static Lifter create(String logPath) {
		return TalonFXLifterBuilder.createLifter(logPath);
	}

}
