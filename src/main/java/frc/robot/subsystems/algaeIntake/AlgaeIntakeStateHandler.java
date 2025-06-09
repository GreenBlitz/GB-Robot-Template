package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.algaeIntake.pivot.PivotState;
import frc.robot.subsystems.algaeIntake.pivot.PivotStateHandler;
import frc.robot.subsystems.algaeIntake.rollers.RollersState;
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
		return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), switch (state) {
			case CLOSED -> close();
			case INTAKE -> intake();
			case TRANSFER_TO_END_EFFECTOR_WITHOUT_RELEASE -> transferToEndEffectorWithoutRelease();
			case TRANSFER_TO_END_EFFECTOR_WITH_RELEASE -> transferToEndEffectorWithRelease();
			case OUTTAKE_WITHOUT_RELEASE -> outtakeWithoutRelease();
			case OUTTAKE_WITH_RELEASE -> outtakeWithRelease();
			case HOLD_ALGAE -> holdAlgae();
			case STAY_IN_PLACE -> stayInPlace();
		});
	}

	public boolean isAtState(AlgaeIntakeState state) {
		return pivotStateHandler.isAtState(getPivotStateByState(state));
	}


	private Command close() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.CLOSED), rollersStateHandler.setState(RollersState.IDLE));
	}

	private Command intake() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.INTAKE), rollersStateHandler.setState(RollersState.INTAKE));
	}

	private Command transferToEndEffectorWithoutRelease() {
		return new ParallelCommandGroup(
			pivotStateHandler.setState(PivotState.HOLD_ALGAE),
			rollersStateHandler.setState(RollersState.TRANSFER_TO_END_EFFECTOR)
		);
	}

	private Command transferToEndEffectorWithRelease() {
		return new ParallelCommandGroup(
			pivotStateHandler.setState(PivotState.TRANSFER_TO_END_EFFECTOR),
			rollersStateHandler.setState(RollersState.TRANSFER_TO_END_EFFECTOR)
		);
	}

	private Command outtakeWithoutRelease() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.OUTTAKE), rollersStateHandler.setState(RollersState.IDLE));
	}

	private Command outtakeWithRelease() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.OUTTAKE), rollersStateHandler.setState(RollersState.OUTTAKE));
	}

	private Command holdAlgae() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.HOLD_ALGAE), rollersStateHandler.setState(RollersState.IDLE));
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.STAY_IN_PLACE), rollersStateHandler.setState(RollersState.IDLE));
	}

	public Command handleIdle() {
		if (rollersStateHandler.isAlgaeIn()) {
			return holdAlgae();
		}
		return close();
	}

	private PivotState getPivotStateByState(AlgaeIntakeState state) {
		return switch (state) {
			case CLOSED -> PivotState.CLOSED;
			case INTAKE -> PivotState.INTAKE;
			case TRANSFER_TO_END_EFFECTOR_WITHOUT_RELEASE, TRANSFER_TO_END_EFFECTOR_WITH_RELEASE -> PivotState.TRANSFER_TO_END_EFFECTOR;
			case OUTTAKE_WITHOUT_RELEASE, OUTTAKE_WITH_RELEASE -> PivotState.OUTTAKE;
			case HOLD_ALGAE -> PivotState.HOLD_ALGAE;
			case STAY_IN_PLACE -> PivotState.STAY_IN_PLACE;
		};
	}

}
