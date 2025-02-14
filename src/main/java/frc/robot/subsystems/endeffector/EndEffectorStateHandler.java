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
		Command stateCommand;
		if (state == EndEffectorState.DEFAULT) {
			stateCommand = endEffector.getCommandsBuilder().setPower(this::defaultStatePower);
		} else {
			stateCommand = endEffector.getCommandsBuilder().setPower(state.getPower());
		}
		return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), stateCommand);
	}

	private double defaultStatePower() {
		return endEffector.isCoralInBack() ? EndEffectorState.DEFAULT.getPower() : -EndEffectorState.DEFAULT.getPower();
	}


}
