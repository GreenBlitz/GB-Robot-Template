package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorStatesHandler {

	private final Elevator elevator;

	public ElevatorStatesHandler(Elevator elevator) {
		this.elevator = elevator;
	}

	public Command setState(ElevatorStates elevatorState) {
		return elevator.getCommandsBuilder().setTargetPositionMeters(elevatorState.getPositionMeters());
	}

}
