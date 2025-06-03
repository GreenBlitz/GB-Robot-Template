package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
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
		return switch (state) {
			case CLOSED -> close();
			case INTAKE -> intake();
			case TRANSFER_TO_END_EFFECTOR -> transferToEndEffector();
			case OUTTAKE -> outtake();
			case STAY_IN_PLACE -> null;
		};
	}


	private Command close() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.CLOSED), rollersStateHandler.setState(RollersState.IDLE));
	}

	private Command intake() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.INTAKE), rollersStateHandler.setState(RollersState.INTAKE));
	}

	private Command transferToEndEffector() {
		return new ParallelCommandGroup(
			pivotStateHandler.setState(PivotState.TRANSFER_TO_END_EFFECTOR),
			rollersStateHandler.setState(RollersState.TRANSFER_TO_END_EFFECTOR)
		);
	}

	private Command outtake() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.OUTTAKE), rollersStateHandler.setState(RollersState.OUTTAKE));
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(pivotStateHandler.setState(PivotState.STAY_IN_PLACE), rollersStateHandler.setState(RollersState.IDLE));
	}

}
