package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.hardware.YishaiDistanceSensor;
import frc.robot.subsystems.algaeIntake.pivot.PivotStateHandler;
import frc.robot.subsystems.algaeIntake.rollers.RollersStateHandler;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntakeStateHandler {

	private final PivotStateHandler pivotStateHandler;
	private final RollersStateHandler rollersStateHandler;

	private final YishaiDistanceSensor distanceSensor;
	private final MedianFilter distanceFilter;

	private AlgaeIntakeState currentState;


	public AlgaeIntakeStateHandler(PivotStateHandler pivotStateHandler, RollersStateHandler rollersStateHandler) {
		this.pivotStateHandler = pivotStateHandler;
		this.rollersStateHandler = rollersStateHandler;

		this.distanceSensor = new YishaiDistanceSensor(new DigitalInput(AlgaeIntakeConstants.ALGAE_SENSOR_CHANNEL));
		this.distanceFilter = new MedianFilter(AlgaeIntakeConstants.NUMBER_OF_VALUES_IN_MEDIAN);
		distanceFilter.reset();
		distanceFilter.calculate(distanceSensor.getDistanceMeters());
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

	public boolean isAtState(AlgaeIntakeState state) {
		return pivotStateHandler.isAtState(state.getPivotState());
	}

	public boolean isAlgaeIn() {
		return !(pivotStateHandler.getPivot().getPosition().getDegrees()
			> AlgaeIntakeConstants.MIN_POSITION_WHEN_CLIMB_INTERRUPT_SENSOR.getDegrees())
			&& distanceFilter.lastValue() < AlgaeIntakeConstants.DISTANCE_FROM_SENSOR_TO_CONSIDER_ALGAE_IN_METERS;
	}

	public Command handleIdle(boolean isAlgaeInAlgaeIntakeOverride) {
		if (isAlgaeIn() || isAlgaeInAlgaeIntakeOverride) {
			return setState(AlgaeIntakeState.HOLD_ALGAE);
		}
		return setState(AlgaeIntakeState.CLOSED);
	}

	public void updateAlgaeSensor(Robot robot) {
		if (
			Math.abs(robot.getPivot().getVelocity().getDegrees())
				< AlgaeIntakeConstants.MAXIMAL_PIVOT_VELOCITY_TO_UPDATE_FILTER_ANGLE_PER_SECOND.getDegrees()
				&& pivotStateHandler.getPivot().getPosition().getDegrees()
					< AlgaeIntakeConstants.MIN_POSITION_WHEN_CLIMB_INTERRUPT_SENSOR.getDegrees()
		) {
			distanceFilter.calculate(distanceSensor.getDistanceMeters());
		} else {
			distanceFilter.calculate(AlgaeIntakeConstants.NO_OBJECT_DEFAULT_DISTANCE);
		}
		Logger.recordOutput(rollersStateHandler.getRollers().getLogPath() + "/DistanceFilterMeters", distanceFilter.lastValue());
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
