package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

	CLOSED(Rotation2d.fromDegrees(210)),
	INTAKE(Rotation2d.fromDegrees(185)),
	OUTTAKE(Rotation2d.fromDegrees(13)),
	PRE_L1(Rotation2d.fromDegrees(215)),
	L1(Rotation2d.fromDegrees(215)),
	PRE_L2(Rotation2d.fromDegrees(-11)),
	L2(Rotation2d.fromDegrees(-11)),
	PRE_L3(Rotation2d.fromDegrees(-4)),
	L3(Rotation2d.fromDegrees(-4)),
	PRE_L4(Rotation2d.fromDegrees(-26)),
	L4(Rotation2d.fromDegrees(-26));

	private final Rotation2d position;

	ArmState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
