package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

	// @formatter:off
	STAY_IN_PLACE(
		Rotation2d.fromDegrees(Double.NaN),
		Rotation2d.fromDegrees(0),
		Rotation2d.fromDegrees(0)
	),
	CLOSED(
		Rotation2d.fromDegrees(180),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED),
	FIRST_WAYPOINT_TO_CLOSE(
		Rotation2d.fromDegrees(40),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	SECOND_WAYPOINT_TO_CLOSE(
		Rotation2d.fromDegrees(110),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	START_GAME(
		Rotation2d.fromDegrees(207),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	MID_WAY_CLOSE(
		Rotation2d.fromDegrees(130),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	INTAKE(Rotation2d.fromDegrees(186), ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND, ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED),
	ALGAE_OUTTAKE(
		Rotation2d.fromDegrees(185),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	PRE_L1(
		Rotation2d.fromDegrees(190),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	L1(
		Rotation2d.fromDegrees(190),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	PRE_L2(
		Rotation2d.fromDegrees(-13),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	L2(
		Rotation2d.fromDegrees(-13),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	PRE_L3(
		Rotation2d.fromDegrees(10),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	L3(
		Rotation2d.fromDegrees(10),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	PRE_L4(
		Rotation2d.fromDegrees(55),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	L4(
		Rotation2d.fromDegrees(-25),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	LOW_ALGAE_REMOVE(
		Rotation2d.fromDegrees(-30),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	HIGH_ALGAE_REMOVE(
		Rotation2d.fromDegrees(-30),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	PRE_NET(
		Rotation2d.fromDegrees(60),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	NET(
		Rotation2d.fromDegrees(45),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	PROCESSOR_OUTTAKE(
		Rotation2d.fromDegrees(180),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	),
	CLIMB(
		Rotation2d.fromDegrees(-11),
		ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND,
		ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED
	);
	// @formatter:on

	private final Rotation2d position;
	private final Rotation2d maxVelocityRotation2dPerSecond;
	private final Rotation2d maxAccelerationRotation2dPerSecondSquared;

	ArmState(Rotation2d position, Rotation2d maxVelocityRotation2dPerSecond, Rotation2d maxAccelerationRotation2dPerSecondSquared) {
		this.position = position;
		this.maxVelocityRotation2dPerSecond = maxVelocityRotation2dPerSecond;
		this.maxAccelerationRotation2dPerSecondSquared = maxAccelerationRotation2dPerSecondSquared;
	}

	public Rotation2d getPosition() {
		return position;
	}

	public Rotation2d getMaxVelocityRotation2dPerSecond() {
		return maxVelocityRotation2dPerSecond;
	}

	public Rotation2d getMaxAccelerationRotation2dPerSecondSquared() {
		return maxAccelerationRotation2dPerSecondSquared;
	}

}
