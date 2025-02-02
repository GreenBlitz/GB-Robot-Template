package frc.robot.statemachine.superstructure;

import frc.robot.statemachine.RobotState;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;

public enum ScoreLevel {

	L1(
		ElevatorState.L1,
		ElevatorState.PRE_L1,
		ArmState.L1,
		ArmState.PRE_L1,
		SuperstructureState.SCORE_L1,
		SuperstructureState.PRE_L1,
		RobotState.L1,
		RobotState.PRE_L1
	),
	L2(
		ElevatorState.L2,
		ElevatorState.PRE_L2,
		ArmState.L2,
		ArmState.PRE_L2,
		SuperstructureState.SCORE_L2,
		SuperstructureState.PRE_L2,
		RobotState.L2,
		RobotState.PRE_L2
	),
	L3(
		ElevatorState.L3,
		ElevatorState.PRE_L3,
		ArmState.L3,
		ArmState.PRE_L3,
		SuperstructureState.SCORE_L3,
		SuperstructureState.PRE_L3,
		RobotState.L3,
		RobotState.PRE_L3
	),
	L4(
		ElevatorState.L4,
		ElevatorState.PRE_L4,
		ArmState.L4,
		ArmState.PRE_L4,
		SuperstructureState.SCORE_L4,
		SuperstructureState.PRE_L4,
		RobotState.L4,
		RobotState.PRE_L4
	);

	private final ElevatorState elevatorScore;
	private final ElevatorState elevatorPreScore;
	private final ArmState armScore;
	private final ArmState armPreScore;
	private final SuperstructureState superstructureScore;
	private final SuperstructureState superstructurePreScore;
	private final RobotState robotScore;
	private final RobotState robotPreScore;

	ScoreLevel(
		ElevatorState elevatorScore,
		ElevatorState elevatorPreScore,
		ArmState armScore,
		ArmState armPreScore,
		SuperstructureState superstructureScore,
		SuperstructureState superstructurePreScore,
		RobotState robotScore,
		RobotState robotPreScore
	) {
		this.elevatorScore = elevatorScore;
		this.elevatorPreScore = elevatorPreScore;
		this.armScore = armScore;
		this.armPreScore = armPreScore;
		this.superstructureScore = superstructureScore;
		this.superstructurePreScore = superstructurePreScore;
		this.robotScore = robotScore;
		this.robotPreScore = robotPreScore;
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

	public SuperstructureState getSuperstructureScore() {
		return superstructureScore;
	}

	public SuperstructureState getSuperstructurePreScore() {
		return superstructurePreScore;
	}

	public RobotState getRobotScore() {
		return robotScore;
	}

	public RobotState getRobotPreScore() {
		return robotPreScore;
	}

}
