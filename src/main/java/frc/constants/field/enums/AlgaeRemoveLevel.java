package frc.constants.field.enums;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.endeffector.EndEffectorState;

public enum AlgaeRemoveLevel {

	LOW(ElevatorState.LOW_ALGAE_REMOVE, ArmState.LOW_ALGAE_REMOVE, EndEffectorState.ALGAE_INTAKE),
	HIGH(ElevatorState.HIGH_ALGAE_REMOVE, ArmState.HIGH_ALGAE_REMOVE, EndEffectorState.ALGAE_INTAKE);

	private final ElevatorState elevatorState;
	private final ArmState armState;
	private final EndEffectorState endEffectorState;

	AlgaeRemoveLevel(ElevatorState elevatorState, ArmState armState, EndEffectorState endEffectorState) {
		this.elevatorState = elevatorState;
		this.armState = armState;
		this.endEffectorState = endEffectorState;
	}

	public ElevatorState getElevatorState() {
		return elevatorState;
	}

	public ArmState getArmState() {
		return armState;
	}

	public EndEffectorState getEndEffectorState() {
		return endEffectorState;
	}

}
