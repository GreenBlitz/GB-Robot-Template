package frc.constants.field.enums;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.endeffector.EndEffectorState;

public enum AlgaeRemoveLevel {

	LOW(
		ElevatorState.LOW_ALGAE_REMOVE,
		ElevatorState.PRE_LOW_ALGAE_REMOVE,
		ArmState.LOW_ALGAE_REMOVE,
		ArmState.PRE_LOW_ALGAE_REMOVE,
		EndEffectorState.ALGAE_INTAKE
	),
	HIGH(
		ElevatorState.HIGH_ALGAE_REMOVE,
		ElevatorState.PRE_HIGH_ALGAE_REMOVE,
		ArmState.HIGH_ALGAE_REMOVE,
		ArmState.HIGH_ALGAE_REMOVE,
		EndEffectorState.ALGAE_INTAKE
	);

	private final ElevatorState elevatorState;
	private final ElevatorState preElevatorState;
	private final ArmState armState;
	private final ArmState preArmState;
	private final EndEffectorState endEffectorState;

	AlgaeRemoveLevel(
		ElevatorState elevatorState,
		ElevatorState preElevatorState,
		ArmState armState,
		ArmState preArmState,
		EndEffectorState endEffectorState
	) {
		this.elevatorState = elevatorState;
		this.preElevatorState = preElevatorState;
		this.armState = armState;
		this.preArmState = preArmState;
		this.endEffectorState = endEffectorState;
	}

	public ElevatorState getElevatorState() {
		return elevatorState;
	}

	public ElevatorState getPreElevatorState() {
		return preElevatorState;
	}

	public ArmState getArmState() {
		return armState;
	}

	public ArmState getPreArmState() {
		return preArmState;
	}

	public EndEffectorState getEndEffectorState() {
		return endEffectorState;
	}

}
