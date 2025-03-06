package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.statemachine.superstructure.Superstructure;

public class EndEffectorStateHandler {

	private static final double CORAL_KEEP_POWER = 0.02;
	private static final double ALGAE_KEEP_POWER = -0.04;

	private final EndEffector endEffector;
	private final Superstructure superstructure;
	private EndEffectorState currentState;

	public EndEffectorStateHandler(EndEffector endEffector, Superstructure superstructure) {
		this.endEffector = endEffector;
		this.superstructure = superstructure;
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
		return superstructure.isCoralIn() ? CORAL_KEEP_POWER : ALGAE_KEEP_POWER;
	}


}
