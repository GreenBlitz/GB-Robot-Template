package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ElevatorState {

	STAY_IN_PLACE(Double.NaN, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
	CLOSED(0.02, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	INTAKE(0.06, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	ALGAE_OUTTAKE(0.02, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	PRE_L1(0, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	L1(0.02, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	PRE_L2(0.02, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	L2(0.02, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	PRE_L3(0.19, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	L3(0.19, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	WHILE_DRIVE_L4(0.4, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	PRE_L4(1.18, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	L4(1.18, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	LOW_ALGAE_REMOVE(0.02, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	HIGH_ALGAE_REMOVE(0.4, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	WHILE_DRIVE_NET(0.4, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	NET(1.18, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100)),
	PROCESSOR_OUTTAKE(0.02, Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(100));

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
