package frc.robot.subsystems.elbow;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ElbowState {

	IDLE(Rotation2d.fromDegrees(-72)),
	INTAKE(Rotation2d.fromDegrees(-78)),
	PRE_AMP(Rotation2d.fromDegrees(50));

	private final Rotation2d targetPosition;

	ElbowState(Rotation2d targetPosition) {
		this.targetPosition = targetPosition;
	}

	public Rotation2d getTargetPosition() {
		return targetPosition;
	}

}
