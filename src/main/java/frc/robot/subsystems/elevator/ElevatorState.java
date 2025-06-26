package frc.robot.subsystems.elevator;


public enum ElevatorState {

	STAY_IN_PLACE(Double.NaN),
	CLOSED(0.02, 6, 6),
	HOLD_ALGAE(0.12, 5, 5),
	INTAKE(0.05),
	ALGAE_OUTTAKE(0.02),
	PRE_L1(0),
	L1(0.02),
	PRE_L2(0.02),
	L2(0.02),
	PRE_L3(0.19),
	L3(0.19),
	WHILE_DRIVE_L4(0.4, 4, 4),
	PRE_L4(1.18, 4, 4),
	L4(1.18, 4, 4),
	LOW_ALGAE_REMOVE(0.02),
	HIGH_ALGAE_REMOVE(0.05),
	TRANSFER_ALGAE_FROM_INTAKE(0.1),
	NET(1.05, 6, 6),
	PROCESSOR_OUTTAKE(0.02),
	OPENING_HEIGHT(0.4, 5, 6),
	CLIMB(0.23);

	private final double heightMeters;
	private final double maxVelocityMetersPerSecond;
	private final double maxAccelerationMetersPerSecondSquared;

	ElevatorState(double heightMeters, double maxVelocityMetersPerSecond, double maxAccelerationMetersPerSecondSquared) {
		this.heightMeters = heightMeters;
		this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
		this.maxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
	}

	ElevatorState(double heightMeters) {
		this(heightMeters, ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND, ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED);
	}

	public double getHeightMeters() {
		return heightMeters;
	}

	public double getMaxVelocityMetersPerSecond() {
		return maxVelocityMetersPerSecond;
	}

	public double getMaxAccelerationMetersPerSecondSquared() {
		return maxAccelerationMetersPerSecondSquared;
	}

}
