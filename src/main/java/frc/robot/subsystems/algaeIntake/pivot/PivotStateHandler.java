package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotStateHandler {

	private final Pivot pivot;
	private PivotState currentState;

	public PivotStateHandler(Pivot pivot) {
		this.pivot = pivot;
	}

	public PivotState getCurrentState() {
		return currentState;
	}

	public Command setState(PivotState state) {
		return pivot.getCommandsBuilder().moveToPosition(state.getPosition());
	}

	public boolean isAtState(PivotState state) {
		return isAtState(state, PivotConstants.TOLERANCE);
	}

	public boolean isAtState(PivotState state, Rotation2d tolerance) {
		return currentState == state && pivot.isAtPosition(state.getPosition(), tolerance);
	}

}
