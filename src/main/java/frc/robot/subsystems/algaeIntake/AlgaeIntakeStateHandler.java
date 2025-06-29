package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.hardware.YishaiDistanceSensor;
import frc.robot.subsystems.algaeIntake.pivot.PivotStateHandler;
import frc.robot.subsystems.algaeIntake.rollers.RollersStateHandler;
import frc.utils.buffers.RingBuffer.RingBuffer;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntakeStateHandler {

	private final PivotStateHandler pivotStateHandler;
	private final RollersStateHandler rollersStateHandler;

	private final RingBuffer<Double> ringBuffer;
	private final YishaiDistanceSensor distanceSensor;

	private AlgaeIntakeState currentState;
	private double min;

	public AlgaeIntakeStateHandler(PivotStateHandler pivotStateHandler, RollersStateHandler rollersStateHandler) {
		this.pivotStateHandler = pivotStateHandler;
		this.rollersStateHandler = rollersStateHandler;

		this.ringBuffer = new RingBuffer<>(AlgaeIntakeConstants.NUMBER_OF_VALUES_IN_MEDIAN);
		this.distanceSensor = new YishaiDistanceSensor(new DigitalInput(AlgaeIntakeConstants.ALGAE_SENSOR_CHANNEL));
		this.min = AlgaeIntakeConstants.NO_OBJECT_DEFAULT_DISTANCE;
	}

	public AlgaeIntakeState getCurrentState() {
		return currentState;
	}

	public Command setState(AlgaeIntakeState state) {
		return new ParallelCommandGroup(new InstantCommand(() -> {
			currentState = state;
			if (state == AlgaeIntakeState.INTAKE) {
				min = AlgaeIntakeConstants.NO_OBJECT_DEFAULT_DISTANCE;
			}
		}), pivotStateHandler.setState(state.getPivotState()), rollersStateHandler.setState(state.getRollersState()));
	}

	public boolean isAtState(AlgaeIntakeState state) {
		return pivotStateHandler.isAtState(state.getPivotState());
	}

	public boolean isAlgaeIn() {
		boolean isPivotDown = pivotStateHandler.getPivot().getPosition().getDegrees()
			< AlgaeIntakeConstants.MIN_POSITION_WHEN_CLIMB_INTERRUPT_SENSOR.getDegrees();
		boolean isAlgaeInByMin = min < AlgaeIntakeConstants.DISTANCE_FROM_SENSOR_TO_CONSIDER_ALGAE_IN_METERS;

		Logger.recordOutput("Test/PivotDown", isPivotDown);
		Logger.recordOutput("Test/MinClose", isAlgaeInByMin);
		Logger.recordOutput("Test/isAlgaeInByMin", isAlgaeInByMin && isPivotDown);

		return isAlgaeInByMin && isPivotDown;
	}

	public Command handleIdle(boolean isAlgaeInAlgaeIntakeOverride) {
		return new ConditionalCommand(
			setState(AlgaeIntakeState.HOLD_ALGAE),
			setState(AlgaeIntakeState.CLOSED),
			() -> isAlgaeIn() || isAlgaeInAlgaeIntakeOverride
		);
	}

	public void updateAlgaeSensor(Robot robot) {
		if (
			Math.abs(robot.getPivot().getVelocity().getDegrees())
				< AlgaeIntakeConstants.MAXIMAL_PIVOT_VELOCITY_TO_UPDATE_FILTER_ANGLE_PER_SECOND.getDegrees()
				&& pivotStateHandler.getPivot().getPosition().getDegrees()
					< AlgaeIntakeConstants.MIN_POSITION_WHEN_CLIMB_INTERRUPT_SENSOR.getDegrees()
		) {
			ringBuffer.insert(distanceSensor.getDistanceMeters());
		} else {
			ringBuffer.insert(AlgaeIntakeConstants.NO_OBJECT_DEFAULT_DISTANCE);
		}

		min = AlgaeIntakeConstants.NO_OBJECT_DEFAULT_DISTANCE;
		ringBuffer.forEach((val) -> min = Math.min(min, val));
		Logger.recordOutput(rollersStateHandler.getRollers().getLogPath() + "/Min", min);
	}


	public void applyCalibrationBindings(SmartJoystick joystick) {
		joystick.A.onTrue(setState(AlgaeIntakeState.CLOSED));
		joystick.B.onTrue(setState(AlgaeIntakeState.INTAKE));
		joystick.Y.onTrue(setState(AlgaeIntakeState.TRANSFER_TO_END_EFFECTOR_WITHOUT_RELEASE));
		joystick.POV_LEFT.onTrue(setState(AlgaeIntakeState.OUTTAKE_WITH_RELEASE));
		joystick.POV_RIGHT.onTrue(setState(AlgaeIntakeState.HOLD_ALGAE));
	}

}
