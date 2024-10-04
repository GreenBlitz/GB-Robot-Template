package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public enum WristState {

	IN_ARM(Rotation2d.fromDegrees(0)),
	ARM_INTAKE(Rotation2d.fromDegrees(-60)),
	PRE_TRAP(Rotation2d.fromDegrees(90)),
	TRAP(Rotation2d.fromDegrees(180));

	private final Rotation2d position;

	WristState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
