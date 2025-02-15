package frc.robot.statemachine.superstructure;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.endeffector.EndEffectorState;

public enum ScoreLevel {

	L1(EndEffectorState.L1_OUTTAKE, ElevatorState.L1, ElevatorState.PRE_L1, ElevatorState.PRE_L1, ArmState.L1, ArmState.PRE_L1),
	L2(EndEffectorState.BRANCH_OUTTAKE, ElevatorState.L2, ElevatorState.PRE_L2, ElevatorState.PRE_L2, ArmState.L2, ArmState.PRE_L2),
	L3(EndEffectorState.BRANCH_OUTTAKE, ElevatorState.L3, ElevatorState.PRE_L3, ElevatorState.PRE_L3, ArmState.L3, ArmState.PRE_L3),
	L4(EndEffectorState.BRANCH_OUTTAKE, ElevatorState.L4, ElevatorState.PRE_L4, ElevatorState.WHILE_DRIVE_L4, ArmState.L4, ArmState.PRE_L4);

	private final EndEffectorState endEffectorScore;
	private final ElevatorState elevatorScore;
	private final ElevatorState elevatorPreScore;
	private final ElevatorState elevatorWhileDrive;
	private final ArmState armScore;
	private final ArmState armPreScore;

	ScoreLevel(
		EndEffectorState endEffectorScore,
		ElevatorState elevatorScore,
		ElevatorState elevatorPreScore,
		ElevatorState elevatorWhileDrive,
		ArmState armScore,
		ArmState armPreScore
	) {
		this.endEffectorScore = endEffectorScore;
		this.elevatorScore = elevatorScore;
		this.elevatorPreScore = elevatorPreScore;
		this.elevatorWhileDrive = elevatorWhileDrive;
		this.armScore = armScore;
		this.armPreScore = armPreScore;
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

	public ElevatorState getElevatorWhileDrive() {
		return elevatorWhileDrive;
	}

	public ArmState getArmScore() {
		return armScore;
	}

	public ArmState getArmPreScore() {
		return armPreScore;
	}

}
