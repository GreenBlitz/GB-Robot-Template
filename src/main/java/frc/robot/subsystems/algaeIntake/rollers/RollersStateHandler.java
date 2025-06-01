package frc.robot.subsystems.algaeIntake.rollers;

import edu.wpi.first.wpilibj2.command.Command;

public class RollersStateHandler {

	private final Rollers rollers;
	private RollersState currentState;

	public RollersStateHandler(Rollers rollers) {
		this.rollers = rollers;
	}

	public RollersState getCurrentState() {
		return currentState;
	}

	public Command setState(RollersState state) {
		return rollers.getCommandsBuilder().setPower(state.getPower());
	}

}
