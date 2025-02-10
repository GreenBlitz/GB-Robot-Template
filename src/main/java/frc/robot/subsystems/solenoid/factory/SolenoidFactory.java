package frc.robot.subsystems.solenoid.factory;

import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.factory.EndEffectorSparkMaxBuilder;

public class SolenoidFactory {

	public static EndEffector create(String logPath) {
		return EndEffectorSparkMaxBuilder.generate(logPath);
	}

}
