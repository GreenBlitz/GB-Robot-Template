package frc.robot.subsystems.elbow;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ElbowState {

	IDLE(Rotation2d.fromDegrees(-72)),
	MANUAL(new Rotation2d()),
	INTAKE(Rotation2d.fromDegrees(-78)),
	ARM_INTAKE(Rotation2d.fromDegrees(-74)),
	PRE_AMP(Rotation2d.fromDegrees(50)),
	TRANSFER(Rotation2d.fromDegrees(-78)),
	PRE_CLIMB(Rotation2d.fromDegrees(-7)),
	CLIMB(Rotation2d.fromDegrees(90)),
	FREE(new Rotation2d());

	private final Rotation2d targetPosition;

	ElbowState(Rotation2d targetPosition) {
		this.targetPosition = targetPosition;
	}

	public Rotation2d getTargetPosition() {
		return targetPosition;
	}

}
