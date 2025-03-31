package frc.robot.subsystems.elevator.factory;

import frc.robot.Robot;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorFactory {

	public static Elevator create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> KrakenX60ElevatorBuilder.createRealElevator(logPath);
			case SIMULATION -> KrakenX60ElevatorBuilder.createSimulationElevator(logPath);
		};
	}

}
