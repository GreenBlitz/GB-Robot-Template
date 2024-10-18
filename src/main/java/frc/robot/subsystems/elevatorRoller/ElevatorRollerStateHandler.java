package frc.robot.subsystems.elevatorRoller;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorRollerStateHandler {

	private final ElevatorRoller elevatorRoller;

	public ElevatorRollerStateHandler(ElevatorRoller elevatorRoller) {
		this.elevatorRoller = elevatorRoller;
	}

	public Command setState(ElevatorRollerState state) {
		return elevatorRoller.getCommandsBuilder().setPower(state.getPower());
	}

}
