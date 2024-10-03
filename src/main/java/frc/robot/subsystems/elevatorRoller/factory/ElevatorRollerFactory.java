package frc.robot.subsystems.elevatorRoller.factory;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerStuff;

public class ElevatorRollerFactory {

	public static ElevatorRollerStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealElevatorRollerConstants.generateRollerStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
