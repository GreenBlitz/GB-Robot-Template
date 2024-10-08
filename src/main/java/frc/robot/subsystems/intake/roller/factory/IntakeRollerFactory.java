package frc.robot.subsystems.intake.roller.factory;

import frc.robot.Robot;
import frc.robot.subsystems.intake.roller.IntakeRollerStuff;

public class IntakeRollerFactory {

	public static IntakeRollerStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> IntakeRollerRealConstant.generateIntakeRollerStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
