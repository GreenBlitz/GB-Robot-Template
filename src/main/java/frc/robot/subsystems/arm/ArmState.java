package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

	INTAKE(Rotation2d.fromDegrees(-115)),
	LOW_DROP(Rotation2d.fromDegrees(-10)),
	MID_DROP(Rotation2d.fromDegrees(65)),
	HIGH_DROP(Rotation2d.fromDegrees(125)),
	SAFE_HOLD(Rotation2d.fromDegrees(-80)),
	STAY_IN_PLACE(Rotation2d.fromDegrees(999));


	private final Rotation2d position;

	ArmState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
