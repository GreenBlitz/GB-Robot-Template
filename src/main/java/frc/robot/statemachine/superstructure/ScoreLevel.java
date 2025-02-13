package frc.robot.statemachine.superstructure;

import frc.robot.statemachine.RobotState;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.endeffector.EndEffectorState;

public enum ScoreLevel {

	L1(
		EndEffectorState.L1_OUTTAKE,
		ElevatorState.L1,
		ElevatorState.PRE_L1,
		ArmState.L1,
		ArmState.PRE_L1,
		SuperstructureState.SCORE_L1,
		SuperstructureState.SCORE_L1_WITHOUT_RELEASE,
		SuperstructureState.PRE_L1,
		SuperstructureState.ARM_PRE_L1,
		RobotState.L1,
		RobotState.L1_WITHOUT_RELEASE,
		RobotState.PRE_L1,
		RobotState.ARM_PRE_L1
	),
	L2(
		EndEffectorState.BRANCH_OUTTAKE,
		ElevatorState.L2,
		ElevatorState.PRE_L2,
		ArmState.L2,
		ArmState.PRE_L2,
		SuperstructureState.SCORE_L2,
		SuperstructureState.SCORE_L2_WITHOUT_RELEASE,
		SuperstructureState.PRE_L2,
		SuperstructureState.ARM_PRE_L2,
		RobotState.L2,
		RobotState.L2_WITHOUT_RELEASE,
		RobotState.PRE_L2,
		RobotState.ARM_PRE_L2
	),
	L3(
		EndEffectorState.BRANCH_OUTTAKE,
		ElevatorState.L3,
		ElevatorState.PRE_L3,
		ArmState.L3,
		ArmState.PRE_L3,
		SuperstructureState.SCORE_L3,
		SuperstructureState.SCORE_L3_WITHOUT_RELEASE,
		SuperstructureState.PRE_L3,
		SuperstructureState.ARM_PRE_L3,
		RobotState.L3,
		RobotState.L3_WITHOUT_RELEASE,
		RobotState.PRE_L3,
		RobotState.ARM_PRE_L3
	),
	L4(
		EndEffectorState.BRANCH_OUTTAKE,
		ElevatorState.L4,
		ElevatorState.PRE_L4,
		ArmState.L4,
		ArmState.PRE_L4,
		SuperstructureState.SCORE_L4,
		SuperstructureState.SCORE_L4_WITHOUT_RELEASE,
		SuperstructureState.PRE_L4,
		SuperstructureState.ARM_PRE_L4,
		RobotState.L4,
		RobotState.L4_WITHOUT_RELEASE,
		RobotState.PRE_L4,
		RobotState.ARM_PRE_L4
	);

	private final EndEffectorState endEffectorScore;
	private final ElevatorState elevatorScore;
	private final ElevatorState elevatorPreScore;
	private final ArmState armScore;
	private final ArmState armPreScore;
	private final SuperstructureState superstructureScore;
	private final SuperstructureState superstructureScoreWithoutRelease;
	private final SuperstructureState superstructurePreScore;
	private final SuperstructureState superstructureArmPreScore;
	private final RobotState robotScore;
	private final RobotState robotScoreWithoutRelease;
	private final RobotState robotPreScore;
	private final RobotState robotArmPreScore;

	ScoreLevel(
		EndEffectorState endEffectorScore,
		ElevatorState elevatorScore,
		ElevatorState elevatorPreScore,
		ArmState armScore,
		ArmState armPreScore,
		SuperstructureState superstructureScore,
		SuperstructureState superstructureScoreWithoutRelease,
		SuperstructureState superstructurePreScore,
		SuperstructureState superstructureArmPreScore,
		RobotState robotScore,
		RobotState robotScoreWithoutRelease,
		RobotState robotPreScore,
		RobotState robotArmPreScore
	) {
		this.endEffectorScore = endEffectorScore;
		this.elevatorScore = elevatorScore;
		this.elevatorPreScore = elevatorPreScore;
		this.armScore = armScore;
		this.armPreScore = armPreScore;
		this.superstructureScore = superstructureScore;
		this.superstructureScoreWithoutRelease = superstructureScoreWithoutRelease;
		this.superstructurePreScore = superstructurePreScore;
		this.superstructureArmPreScore = superstructureArmPreScore;
		this.robotScore = robotScore;
		this.robotScoreWithoutRelease = robotScoreWithoutRelease;
		this.robotPreScore = robotPreScore;
		this.robotArmPreScore = robotArmPreScore;
	}

	public EndEffectorState getEndEffectorScore() {
		return endEffectorScore;
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

	public SuperstructureState getSuperstructureScoreWithoutRelease() {
		return superstructureScoreWithoutRelease;
	}

	public SuperstructureState getSuperstructurePreScore() {
		return superstructurePreScore;
	}

	public SuperstructureState getSuperstructureArmPreScore() {
		return superstructureArmPreScore;
	}

	public RobotState getRobotScore() {
		return robotScore;
	}

	public RobotState getRobotScoreWithoutRelease() {
		return robotScoreWithoutRelease;
	}

	public RobotState getRobotPreScore() {
		return robotPreScore;
	}

	public RobotState getRobotArmPreScore() {
		return robotArmPreScore;
	}

}
