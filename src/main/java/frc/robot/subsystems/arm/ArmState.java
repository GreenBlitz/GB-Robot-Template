package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

	CLOSED(Rotation2d.fromDegrees(197)),
	INTAKE(Rotation2d.fromDegrees(197)),
	OUTTAKE(Rotation2d.fromDegrees(13)),
	PRE_L1(Rotation2d.fromDegrees(0)),
	PRE_L2(Rotation2d.fromDegrees(13)),
	PRE_L3(Rotation2d.fromDegrees(22)),
	PRE_L4(Rotation2d.fromDegrees(22)),
	L1(Rotation2d.fromDegrees(0)),
	L2(Rotation2d.fromDegrees(13)),
	L3(Rotation2d.fromDegrees(22)),
	L4(Rotation2d.fromDegrees(-20));

	private final Rotation2d position;

	ArmState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
