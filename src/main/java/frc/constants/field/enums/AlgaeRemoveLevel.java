package frc.constants.field.enums;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.endeffector.EndEffectorState;

public enum AlgaeRemoveLevel {

	LOW(
		ElevatorState.LOW_ALGAE_REMOVE,
		ElevatorState.POST_LOW_ALGAE_REMOVE,
		ArmState.LOW_ALGAE_REMOVE,
		ArmState.POST_LOW_ALGAE_REMOVE,
		EndEffectorState.ALGAE_INTAKE
	),
	HIGH(
		ElevatorState.HIGH_ALGAE_REMOVE,
		ElevatorState.POST_HIGH_ALGAE_REMOVE,
		ArmState.HIGH_ALGAE_REMOVE,
		ArmState.PRE_HIGH_ALGAE_REMOVE,
		EndEffectorState.ALGAE_INTAKE
	);

	private final ElevatorState elevatorState;
	private final ElevatorState postElevatorState;
	private final ArmState armState;
	private final ArmState postArmState;
	private final EndEffectorState endEffectorState;

	AlgaeRemoveLevel(
		ElevatorState elevatorState,
		ElevatorState postElevatorState,
		ArmState armState,
		ArmState postArmState,
		EndEffectorState endEffectorState
	) {
		this.elevatorState = elevatorState;
		this.postElevatorState = postElevatorState;
		this.armState = armState;
		this.postArmState = postArmState;
		this.endEffectorState = endEffectorState;
	}

	public ElevatorState getElevatorState() {
		return elevatorState;
	}

	public ElevatorState getPostElevatorState() {
		return postElevatorState;
	}

	public ArmState getArmState() {
		return armState;
	}

	public ArmState getPostArmState() {
		return postArmState;
	}

	public EndEffectorState getEndEffectorState() {
		return endEffectorState;
	}

}
