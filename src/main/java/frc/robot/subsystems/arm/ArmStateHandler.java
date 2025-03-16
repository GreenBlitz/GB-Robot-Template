package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.statemachine.Tolerances;

import java.util.function.Supplier;

public class ArmStateHandler {

	private final Arm arm;
	private ArmState currentState;
	private final Supplier<Double> distanceSupplier;

	public ArmStateHandler(Arm arm, Supplier<Double> distanceSupplier) {
		this.arm = arm;
		this.distanceSupplier = distanceSupplier;
	}


	public ArmState getCurrentState() {
		return currentState;
	}

	public Command setState(ArmState state) {
		return switch (state) {
			case STAY_IN_PLACE ->
				new ParallelCommandGroup(new InstantCommand(() -> currentState = state), arm.getCommandsBuilder().stayInPlace());
			case L2, L3, L4 ->
				new ParallelCommandGroup(
					new InstantCommand(() -> currentState = state),
					arm.getCommandsBuilder()
						.moveToPosition(
							() -> getStatePosition(state),
							state.getMaxVelocityRotation2dPerSecond(),
							state.getMaxAccelerationRotation2dPerSecondSquared()
						)
				);
			default ->
				new ParallelCommandGroup(
					new InstantCommand(() -> currentState = state),
					arm.getCommandsBuilder()
						.moveToPosition(
							getStatePosition(state),
							state.getMaxVelocityRotation2dPerSecond(),
							state.getMaxAccelerationRotation2dPerSecondSquared()
						)
				);
		};
	}

	public boolean isAtState(ArmState state) {
		return isAtState(state, Tolerances.ARM_POSITION);
	}

	public boolean isAtState(ArmState state, Rotation2d tolerance) {
		return arm.isAtPosition(getStatePosition(state), tolerance) && currentState == state;
	}

	private Rotation2d getStatePosition(ArmState state) {
		if (state == ArmState.L4) {
			return Rotation2d
				.fromDegrees(state.getPosition().getDegrees() + ArmConstants.L4_DISTANCE_ANGLE_MAP.get(distanceSupplier.get()).getDegrees());
		} else if (state == ArmState.L2 || state == ArmState.L3) {
			return Rotation2d
				.fromDegrees(state.getPosition().getDegrees() + ArmConstants.L3_L2_DISTANCE_ANGLE_MAP.get(distanceSupplier.get()).getDegrees());
		}
		return state.getPosition();
	}

}
