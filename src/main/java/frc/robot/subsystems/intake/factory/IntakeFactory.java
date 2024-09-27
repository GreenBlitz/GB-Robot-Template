package frc.robot.subsystems.intake.factory;

import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeStuff;

public class IntakeFactory {

	public static IntakeStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealIntakeConstants.generateIntakeStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
