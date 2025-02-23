package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

	STAY_IN_PLACE(Rotation2d.fromDegrees(Double.NaN)),
	CLOSED(Rotation2d.fromDegrees(185)),
	MID_WAY_CLOSE(Rotation2d.fromDegrees(150)),
	INTAKE(Rotation2d.fromDegrees(185)),
	ALGAE_OUTTAKE(Rotation2d.fromDegrees(180)),
	PRE_L1(Rotation2d.fromDegrees(185)),
	L1(Rotation2d.fromDegrees(185)),
	PRE_L2(Rotation2d.fromDegrees(-11)),
	L2(Rotation2d.fromDegrees(-11)),
	PRE_L3(Rotation2d.fromDegrees(10)),
	L3(Rotation2d.fromDegrees(10)),
	PRE_L4(Rotation2d.fromDegrees(37)),
	L4(Rotation2d.fromDegrees(-25)),
	POST_LOW_ALGAE_REMOVE(Rotation2d.fromDegrees(-30)),
	LOW_ALGAE_REMOVE(Rotation2d.fromDegrees(-30)),
	PRE_HIGH_ALGAE_REMOVE(Rotation2d.fromDegrees(1)),
	HIGH_ALGAE_REMOVE(Rotation2d.fromDegrees(1)),
	PRE_NET(Rotation2d.fromDegrees(45)),
	NET(Rotation2d.fromDegrees(45)),
	PROCESSOR_OUTTAKE(Rotation2d.fromDegrees(180));


	private final Rotation2d position;

	ArmState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
