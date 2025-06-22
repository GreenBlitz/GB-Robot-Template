package frc.robot.subsystems.algaeIntake.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RollersStateHandler {

	private final Rollers rollers;
	private RollersState currentState;

	public RollersStateHandler(Rollers rollers) {
		this.rollers = rollers;
	}

	public RollersState getCurrentState() {
		return currentState;
	}

	public Rollers getRollers() {
		return rollers;
	}

	public Command setState(RollersState state) {
		return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), rollers.getCommandsBuilder().setPower(state.getPower()));
	}

}
