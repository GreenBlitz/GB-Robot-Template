package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.algaeIntake.pivot.PivotStateHandler;
import frc.robot.subsystems.algaeIntake.rollers.RollersStateHandler;

public class AlgaeIntakeStateHandler {

	private final PivotStateHandler pivotStateHandler;
	private final RollersStateHandler rollersStateHandler;

	private AlgaeIntakeState currentState;

	public AlgaeIntakeStateHandler(PivotStateHandler pivotStateHandler, RollersStateHandler rollersStateHandler) {
		this.pivotStateHandler = pivotStateHandler;
		this.rollersStateHandler = rollersStateHandler;
	}

	public AlgaeIntakeState getCurrentState() {
		return currentState;
	}

	public Command setState(AlgaeIntakeState state) {
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = state),
			pivotStateHandler.setState(state.getPivotState()),
			rollersStateHandler.setState(state.getRollersState())
		);
	}

	public Command handleIdle() {
		if (rollersStateHandler.isAlgaeIn()) {
			return setState(AlgaeIntakeState.HOLD_ALGAE);
		}
		return setState(AlgaeIntakeState.CLOSED);
	}

}
