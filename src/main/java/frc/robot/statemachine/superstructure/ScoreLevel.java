package frc.robot.statemachine.superstructure;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;

public enum ScoreLevel {

	L1(ElevatorState.L1, ArmState.L1, ArmState.PRE_L1),
	L2(ElevatorState.L2, ArmState.L2, ArmState.PRE_L2),
	L3(ElevatorState.L3, ArmState.L3, ArmState.PRE_L3),
	L4(ElevatorState.L4, ArmState.L4, ArmState.PRE_L4);

	private final ElevatorState elevatorState;
	private final ArmState armState;
	private final ArmState armPreScoreState;

	ScoreLevel(ElevatorState elevatorState, ArmState armState, ArmState armPreScoreState) {
		this.elevatorState = elevatorState;
		this.armState = armState;
		this.armPreScoreState = armPreScoreState;
	}

	public ElevatorState getElevatorState() {
		return elevatorState;
	}

	public ArmState getArmState() {
		return armState;
	}

	public ArmState getArmPreScoreState() {
		return armPreScoreState;
	}

}
