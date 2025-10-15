package frc.robot.statemachine;

import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;

public enum RobotState {

	STAY_IN_PLACE,
	DRIVE,
	INTAKE,
	CORAL_OUTTAKE,
	ELEVATOR_OPENING,
	ARM_PRE_SCORE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
	PRE_SCORE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
	SCORE_WITHOUT_RELEASE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
	SCORE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
	ALGAE_REMOVE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.ALGAE_REMOVE)),
	ALGAE_OUTTAKE_FROM_END_EFFECTOR,
	ALGAE_OUTTAKE_FROM_INTAKE,
	ALGAE_INTAKE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.ALGAE_INTAKE)),
	TRANSFER_ALGAE_TO_END_EFFECTOR,
	HOLD_ALGAE,
	PRE_NET,
	NET,
	PROCESSOR_SCORE,
	PROCESSOR_NO_SCORE,
	PRE_CLIMB(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CAGE)),
	CLIMB_WITHOUT_LIMIT_SWITCH,
	CLIMB_WITH_LIMIT_SWITCH,
	MANUAL_CLIMB,
	EXIT_CLIMB,
	STOP_CLIMB,
	CLOSE_CLIMB;

	private final SwerveState swerveState;
	public boolean swerveStateActive;

	RobotState() {
		this(SwerveState.DEFAULT_DRIVE);
	}

	RobotState(SwerveState state) {
		this.swerveState = state;
		this.swerveStateActive = true;
	}

	public RobotState activateSwerve(boolean active) {
		swerveStateActive = active;
		return this;
	}

	public SwerveState getSwerveState() {
		return swerveStateActive ? swerveState : SwerveState.DEFAULT_DRIVE;
	}

}
