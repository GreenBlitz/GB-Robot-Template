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
		return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), switch (state) {
			case STAY_IN_PLACE -> arm.getCommandsBuilder().stayInPlace();
			case L2, L3, L4, PRE_L3, PRE_L2 ->
				arm.getCommandsBuilder()
					.moveToPosition(
						() -> getStatePosition(state),
						state.getMaxVelocityRotation2dPerSecond(),
						state.getMaxAccelerationRotation2dPerSecondSquared()
					);
			default ->
				arm.getCommandsBuilder()
					.moveToPosition(
						state.getPosition(),
						state.getMaxVelocityRotation2dPerSecond(),
						state.getMaxAccelerationRotation2dPerSecondSquared()
					);
		});
	}

	public boolean isAtState(ArmState state) {
		return isAtState(state, Tolerances.ARM_POSITION);
	}

	public boolean isAtState(ArmState state, Rotation2d tolerance) {
		return arm.isAtPosition(getStatePosition(state), tolerance) && currentState == state;
	}

	private Rotation2d getStatePosition(ArmState state) {
		return Rotation2d.fromDegrees(state.getPosition().getDegrees() + switch (state) {
			case L4 -> ArmConstants.L4_DISTANCE_ANGLE_MAP.get(distanceSupplier.get()).getDegrees();
			case L3, PRE_L3 -> ArmConstants.L3_DISTANCE_ANGLE_MAP.get(distanceSupplier.get()).getDegrees();
			case L2, PRE_L2 -> ArmConstants.L2_DISTANCE_ANGLE_MAP.get(distanceSupplier.get()).getDegrees();
			default -> 0;
		});
	}

}
