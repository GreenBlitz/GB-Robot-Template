package frc.robot.subsystems.elevatorRoller.factory;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerAvatiach;

public class ElevatorRollerFactory {

	public static ElevatorRollerAvatiach create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealElevatorRollerConstants.generate(logPath);
			case SIMULATION -> null;
		};
	}

}
