package frc.robot.statemachine;

import frc.robot.subsystems.swerve.states.SwerveState;

public enum RobotState {

	DRIVE;

	private final SwerveState swerveState;
	private boolean swerveStateActive;

	RobotState() {
		this(SwerveState.DEFAULT_DRIVE);
	}

	RobotState(SwerveState state) {
		this.swerveState = state;
		this.swerveStateActive = true;
	}

	public RobotState activateSwerveAssist(boolean active) {
		swerveStateActive = active;
		return this;
	}

	public SwerveState getSwerveState() {
		return swerveStateActive ? swerveState : SwerveState.DEFAULT_DRIVE;
	}

}
