package frc.robot.subsystems.elevator.factories;

import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorStuff;

public class ElevatorFactory {

	public static ElevatorStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealElevatorConstants.generateElevatorStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
