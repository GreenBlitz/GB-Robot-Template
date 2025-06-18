package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.joysticks.SmartJoystick;
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
		if (state == AlgaeIntakeState.INTAKE) {
			return new ParallelCommandGroup(
				new InstantCommand(() -> currentState = state),
				pivotStateHandler.setState(state.getPivotState()),
				rollersStateHandler.setState(state.getRollersState())
			).until(rollersStateHandler::isAlgaeIn);
		}
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = state),
			pivotStateHandler.setState(state.getPivotState()),
			rollersStateHandler.setState(state.getRollersState())
		);
	}

	public boolean isAtState(AlgaeIntakeState state) {
		return pivotStateHandler.isAtState(state.getPivotState());
	}


	public Command handleIdle(boolean isAlgaeInAlgaeIntakeOverride) {
		if (rollersStateHandler.isAlgaeIn() || isAlgaeInAlgaeIntakeOverride) {
			return setState(AlgaeIntakeState.HOLD_ALGAE);
		}
		return setState(AlgaeIntakeState.CLOSED);
	}


	public void applyCalibrationBindings(SmartJoystick joystick) {
		joystick.A.onTrue(setState(AlgaeIntakeState.CLOSED));
		joystick.B.onTrue(setState(AlgaeIntakeState.INTAKE));
		joystick.X.onTrue(setState(AlgaeIntakeState.OUTTAKE_WITHOUT_RELEASE));
		joystick.Y.onTrue(setState(AlgaeIntakeState.TRANSFER_TO_END_EFFECTOR_WITHOUT_RELEASE));
		joystick.POV_LEFT.onTrue(setState(AlgaeIntakeState.OUTTAKE_WITH_RELEASE));
		joystick.POV_RIGHT.onTrue(setState(AlgaeIntakeState.HOLD_ALGAE));
	}

}
