package frc.robot.subsystems.elevatorRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorRollerStateHandler {

	private final ElevatorRoller elevatorRoller;

	public ElevatorRollerStateHandler(Robot robot) {
		this.elevatorRoller = robot.getElevatorRoller();
	}

	public Command setState(ElevatorRollerState state) {
		return elevatorRoller.getCommandsBuilder().setPower(state.getPower());
	}

}
