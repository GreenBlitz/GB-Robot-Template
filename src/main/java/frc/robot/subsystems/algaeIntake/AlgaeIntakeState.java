package frc.robot.subsystems.algaeIntake;

import frc.robot.subsystems.algaeIntake.pivot.PivotState;
import frc.robot.subsystems.algaeIntake.rollers.RollersState;

public enum AlgaeIntakeState {

	CLOSED(PivotState.CLOSED, RollersState.IDLE),
	INTAKE(PivotState.INTAKE, RollersState.INTAKE),
	TRANSFER_TO_END_EFFECTOR_WITHOUT_RELEASE(PivotState.TRANSFER_TO_END_EFFECTOR, RollersState.IDLE),
	TRANSFER_TO_END_EFFECTOR_WITH_RELEASE(PivotState.TRANSFER_TO_END_EFFECTOR, RollersState.TRANSFER_TO_END_EFFECTOR),
	OUTTAKE_WITHOUT_RELEASE(PivotState.OUTTAKE, RollersState.IDLE),
	OUTTAKE_WITH_RELEASE(PivotState.OUTTAKE, RollersState.OUTTAKE),
	HOLD_ALGAE(PivotState.HOLD_ALGAE, RollersState.IDLE),
	STAY_IN_PLACE(PivotState.STAY_IN_PLACE, RollersState.IDLE);


	private final PivotState pivotState;
	private final RollersState rollersState;

	AlgaeIntakeState(PivotState pivotState, RollersState rollersState) {
		this.pivotState = pivotState;
		this.rollersState = rollersState;
	}

	public PivotState getPivotState() {
		return pivotState;
	}

	public RollersState getRollersState() {
		return rollersState;
	}

}
