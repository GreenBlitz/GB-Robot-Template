package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

	STAY_IN_PLACE(Rotation2d.fromDegrees(Double.NaN), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
	CLOSED(Rotation2d.fromDegrees(180)),
	FIRST_WAYPOINT_TO_CLOSE(Rotation2d.fromDegrees(40)),
	SECOND_WAYPOINT_TO_CLOSE(Rotation2d.fromDegrees(110)),
	START_GAME(Rotation2d.fromDegrees(207)),
	MID_WAY_CLOSE(Rotation2d.fromDegrees(130)),
	INTAKE(Rotation2d.fromDegrees(186)),
	ALGAE_OUTTAKE(Rotation2d.fromDegrees(185)),
	PRE_L1(Rotation2d.fromDegrees(190)),
	L1(Rotation2d.fromDegrees(190)),
	PRE_L2(Rotation2d.fromDegrees(-13)),
	L2(Rotation2d.fromDegrees(-13)),
	PRE_L3(Rotation2d.fromDegrees(10)),
	L3(Rotation2d.fromDegrees(10)),
	PRE_L4(Rotation2d.fromDegrees(55), Rotation2d.fromRotations(3), Rotation2d.fromRotations(0.5)),
	L4(Rotation2d.fromDegrees(-25)),
	LOW_ALGAE_REMOVE(Rotation2d.fromDegrees(-30)),
	HIGH_ALGAE_REMOVE(Rotation2d.fromDegrees(-30)),
	PRE_NET(Rotation2d.fromDegrees(60)),
	NET(Rotation2d.fromDegrees(45), Rotation2d.fromRotations(1), Rotation2d.fromRotations(2)),
	PROCESSOR_OUTTAKE(Rotation2d.fromDegrees(180)),
	CLIMB(Rotation2d.fromDegrees(-11), Rotation2d.fromRotations(0.5), Rotation2d.fromRotations(0.5));

	private final Rotation2d position;
	private final Rotation2d maxVelocityRotation2dPerSecond;
	private final Rotation2d maxAccelerationRotation2dPerSecondSquared;

	ArmState(Rotation2d position, Rotation2d maxVelocityRotation2dPerSecond, Rotation2d maxAccelerationRotation2dPerSecondSquared) {
		this.position = position;
		this.maxVelocityRotation2dPerSecond = maxVelocityRotation2dPerSecond;
		this.maxAccelerationRotation2dPerSecondSquared = maxAccelerationRotation2dPerSecondSquared;
	}

	ArmState(Rotation2d position) {
		this(position, ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND, ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED);
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
