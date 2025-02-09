package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class EndEffectorStateHandler {

	private final EndEffector endEffector;
	private EndEffectorState currentState;

	public EndEffectorStateHandler(EndEffector endEffector) {
		this.endEffector = endEffector;
	}

	public EndEffectorState getCurrentState() {
		return currentState;
	}

	public Command setState(EndEffectorState state) {
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = state),
			endEffector.getCommandsBuilder().setPower(state.getPower())
		);
	}

}
