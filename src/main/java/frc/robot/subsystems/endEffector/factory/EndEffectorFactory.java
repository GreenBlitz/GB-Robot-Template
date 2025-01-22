package frc.robot.subsystems.endEffector.factory;

import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorConstants;

public class EndEffectorFactory {

	public static EndEffector create(String logPath) {
		return EndEffectorSparkMaxBuilder.generate(logPath);
	}

}
