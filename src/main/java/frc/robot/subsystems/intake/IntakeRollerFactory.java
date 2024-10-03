package frc.robot.subsystems.intake;

import frc.robot.Robot;

public class IntakeRollerFactory {

	public static IntakeRollerStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> IntakeRollerRealConstant.generateIntakeRollerStuff(logPath);

			case SIMULATION -> null;
		};
	}

}
