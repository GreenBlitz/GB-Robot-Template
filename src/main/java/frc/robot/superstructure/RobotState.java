package frc.robot.superstructure;

public enum RobotState {

	IDLE, // flywheel, pivot, arm
	PRE_AMP, // swerve, arm
	PRE_SPEAKER, // flywheel, pivot
	TRANSFER_SHOOTER_ARM, // SEQUENCE: funnel, roller, arm, pivot
	TRANSFER_ARM_SHOOTER, // SEQUENCE: funnel, roller, arm, pivot
	INTAKE,
	SHOOTER_OUTTAKE, // intake, pivot, funnel
	SPEAKER, // Funnel
	AMP, // Roller
	PRE_CLIMB,
	CLIMB,
}
