package frc.robot.statemachine.superstructure;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;

public enum ScoreLevel {

	L1(ElevatorState.L1, ElevatorState.PRE_L1, ArmState.L1, ArmState.PRE_L1),
	L2(ElevatorState.L2, ElevatorState.PRE_L2, ArmState.L2, ArmState.PRE_L2),
	L3(ElevatorState.L3, ElevatorState.PRE_L3, ArmState.L3, ArmState.PRE_L3),
	L4(ElevatorState.L4, ElevatorState.PRE_L4, ArmState.L4, ArmState.PRE_L4);

	private final ElevatorState elevatorScore;
	private final ElevatorState elevatorPreScore;
	private final ArmState armScore;
	private final ArmState armPreScore;

	ScoreLevel(ElevatorState elevatorScore, ElevatorState elevatorPreScore, ArmState armScore, ArmState armPreScore) {
		this.elevatorScore = elevatorScore;
		this.elevatorPreScore = elevatorPreScore;
		this.armScore = armScore;
		this.armPreScore = armPreScore;
	}

	public ElevatorState getElevatorScore() {
		return elevatorScore;
	}

	public ElevatorState getElevatorPreScore() {
		return elevatorPreScore;
	}

	public ArmState getArmScore() {
		return armScore;
	}

	public ArmState getArmPreScore() {
		return armPreScore;
	}

}
