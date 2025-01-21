package frc.robot.subsystems.elevator.factory;

import frc.robot.Robot;

public class ElevatorFactory {

	public Elevator generateElevator(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> KrakenElevatorBuilder.create(logPath);
		};
	}

}
