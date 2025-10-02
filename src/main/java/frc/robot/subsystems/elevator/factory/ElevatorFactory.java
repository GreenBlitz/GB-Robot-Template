package frc.robot.subsystems.elevator.factory;

import frc.robot.subsystems.elevator.Elevator;

public class ElevatorFactory {

	public static Elevator create(String logPath) {
		return KrakenX60ElevatorBuilder.create(logPath);
	}

}
