package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public enum PivotState {

	IDLE(Rotation2d.fromDegrees(40)),
	INTAKE(Rotation2d.fromDegrees(35)),
	ARM_INTAKE(Rotation2d.fromDegrees(30)),
	PRE_SPEAKER(Rotation2d.fromDegrees(60)),
	TRANSFER(Rotation2d.fromDegrees(17));

	private final Rotation2d targetPosition;

	PivotState(Rotation2d targetPosition) {
		this.targetPosition = targetPosition;
	}

	public Rotation2d getTargetPosition() {
		return targetPosition;
	}

}
