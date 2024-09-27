package frc.robot.subsystems.intake.factory;

import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeStuff;

public class IntakeFactory {

	public static IntakeStuff create() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealIntakeConstants.generateIntake();
			case SIMULATION -> null;
		};
	}

}
