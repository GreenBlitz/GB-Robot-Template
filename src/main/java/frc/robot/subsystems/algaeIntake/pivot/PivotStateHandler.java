package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.statemachine.Tolerances;

public class PivotStateHandler {

	private final Pivot pivot;
	private PivotState currentState;

	public PivotStateHandler(Pivot pivot) {
		this.pivot = pivot;
	}

	public PivotState getCurrentState() {
		return currentState;
	}

	public Pivot getPivot() {
		return pivot;
	}

	public Command setState(PivotState state) {
		if (state == PivotState.STAY_IN_PLACE) {
			return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), pivot.getCommandsBuilder().stayInPlace());
		}
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = state),
			pivot.getCommandsBuilder().moveToPosition(state.getPosition())
		);
	}

	public boolean isAtState(PivotState state) {
		return isAtState(state, Tolerances.PIVOT);
	}

	public boolean isAtState(PivotState state, Rotation2d tolerance) {
		if (state == PivotState.STAY_IN_PLACE) {
			return currentState == state;
		}
		return currentState == state && pivot.isAtPosition(state.getPosition(), tolerance);
	}

}
