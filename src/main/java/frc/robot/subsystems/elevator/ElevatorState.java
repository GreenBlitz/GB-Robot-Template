package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ElevatorState {

	STAY_IN_PLACE(Double.NaN, Rotation2d.fromRotations(0), Rotation2d.fromDegrees(0)),
	CLOSED(
		0.02,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	INTAKE(
		0.06,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	ALGAE_OUTTAKE(
		0.02,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	PRE_L1(
		0,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	L1(
		0.02,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	PRE_L2(
		0.02,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	L2(
		0.02,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	PRE_L3(
		0.19,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	L3(
		0.19,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	WHILE_DRIVE_L4(
		0.4,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	PRE_L4(
		1.18,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	L4(
		1.18,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	LOW_ALGAE_REMOVE(
		0.02,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	HIGH_ALGAE_REMOVE(
		0.4,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	WHILE_DRIVE_NET(
		0.4,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	NET(
		1.18,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	),
	PROCESSOR_OUTTAKE(
		0.02,
		Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND),
		Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED)
	);

	private final double heightMeters;
	private final Rotation2d maxVelocityRotation2dPerSecond;
	private final Rotation2d maxAccelerationRotation2dPerSecondSquared;

	ElevatorState(double heightMeters, Rotation2d maxVelocityRotation2dPerSecond, Rotation2d maxAccelerationRotation2dPerSecondSquared) {
		this.heightMeters = heightMeters;
		this.maxVelocityRotation2dPerSecond = maxVelocityRotation2dPerSecond;
		this.maxAccelerationRotation2dPerSecondSquared = maxAccelerationRotation2dPerSecondSquared;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

	public Rotation2d getMaxVelocityRotation2dPerSecond() {
		return maxVelocityRotation2dPerSecond;
	}

	public Rotation2d getMaxAccelerationRotation2dPerSecondSquared() {
		return maxAccelerationRotation2dPerSecondSquared;
	}

}
