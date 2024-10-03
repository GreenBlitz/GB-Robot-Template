package frc.robot.subsystems.intake.pivot.factory;

import frc.robot.Robot;
import frc.robot.subsystems.intake.pivot.PivotStuff;

public class PivotFactory {

	public static PivotStuff create() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealPivotConstants.generatePivotStuff();
			case SIMULATION -> null;
		};
	}

}
