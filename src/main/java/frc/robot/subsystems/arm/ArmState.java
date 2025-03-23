package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

	STAY_IN_PLACE(Rotation2d.fromDegrees(Double.NaN)),
	CLOSED(Rotation2d.fromDegrees(219.5)),
	START_GAME(Rotation2d.fromDegrees(233)),
	MID_WAY_CLOSE(Rotation2d.fromDegrees(90), Rotation2d.fromRotations(2), Rotation2d.fromRotations(1.5)),
	INTAKE(Rotation2d.fromDegrees(219.5)),
	ALGAE_OUTTAKE(Rotation2d.fromDegrees(211)),
	PRE_L1(Rotation2d.fromDegrees(216)),
	L1(Rotation2d.fromDegrees(216)),
	PRE_L2(Rotation2d.fromDegrees(14)),
	L2(Rotation2d.fromDegrees(14)),
	PRE_L3(Rotation2d.fromDegrees(36)),
	L3(Rotation2d.fromDegrees(36)),
	PRE_L4(Rotation2d.fromDegrees(81), Rotation2d.fromRotations(3), Rotation2d.fromRotations(1.5)),
	L4(Rotation2d.fromDegrees(-3), Rotation2d.fromRotations(3), Rotation2d.fromRotations(1.5)),
	LOW_ALGAE_REMOVE(Rotation2d.fromDegrees(-4)),
	HIGH_ALGAE_REMOVE(Rotation2d.fromDegrees(35)),
	HOLD_ALGAE(Rotation2d.fromDegrees(90.5)),
	PRE_NET(Rotation2d.fromDegrees(90.5), Rotation2d.fromRotations(2), Rotation2d.fromRotations(2)),
	NET(Rotation2d.fromDegrees(36), Rotation2d.fromRotations(2), Rotation2d.fromRotations(2)),
	PROCESSOR_OUTTAKE(Rotation2d.fromDegrees(206), Rotation2d.fromRotations(2), Rotation2d.fromRotations(1.5)),
	CLIMB(Rotation2d.fromDegrees(15));

	private final Rotation2d position;
	private final Rotation2d maxVelocityRotation2dPerSecond;
	private final Rotation2d maxAccelerationRotation2dPerSecondSquared;

	ArmState(Rotation2d position, Rotation2d maxVelocityRotation2dPerSecond, Rotation2d maxAccelerationRotation2dPerSecondSquared) {
		this.position = Rotation2d.fromDegrees(position.getDegrees() + ArmConstants.POSITION_OFFSET.getDegrees());
		this.maxVelocityRotation2dPerSecond = maxVelocityRotation2dPerSecond;
		this.maxAccelerationRotation2dPerSecondSquared = maxAccelerationRotation2dPerSecondSquared;
	}

	ArmState(Rotation2d position) {
		this(position, ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND, ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED);
	}

	Rotation2d getPosition() {
		return position;
	}

	public Rotation2d getMaxVelocityRotation2dPerSecond() {
		return maxVelocityRotation2dPerSecond;
	}

	public Rotation2d getMaxAccelerationRotation2dPerSecondSquared() {
		return maxAccelerationRotation2dPerSecondSquared;
	}

}
