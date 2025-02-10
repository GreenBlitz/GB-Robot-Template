package frc.robot.subsystems.lifter.factory;

import frc.robot.Robot;
import frc.robot.subsystems.lifter.Lifter;

public class LifterFactory {

	public static Lifter create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> new Lifter(LifterRealConstants.generateLifterStuff(logPath));
			case SIMULATION -> null;
		};
	}

}
