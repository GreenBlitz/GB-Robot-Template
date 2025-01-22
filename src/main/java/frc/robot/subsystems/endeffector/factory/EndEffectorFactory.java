package frc.robot.subsystems.endeffector.factory;

import frc.robot.subsystems.endeffector.EndEffector;

public class EndEffectorFactory {

	public static EndEffector create(String logPath) {
		return EndEffectorSparkMaxBuilder.generate(logPath);
	}

}
