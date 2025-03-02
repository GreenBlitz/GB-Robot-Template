package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

	STAY_IN_PLACE(Rotation2d.fromDegrees(Double.NaN)),
	CLOSED(Rotation2d.fromDegrees(180)),
	FIRST_WAYPOINT_TO_CLOSE(Rotation2d.fromDegrees(40)),
	SECOND_WAYPOINT_TO_CLOSE(Rotation2d.fromDegrees(110)),
	START_GAME(Rotation2d.fromDegrees(207)),
	MID_WAY_CLOSE(Rotation2d.fromDegrees(130)),
	INTAKE(Rotation2d.fromDegrees(186)),
	ALGAE_OUTTAKE(Rotation2d.fromDegrees(185)),
	PRE_L1(Rotation2d.fromDegrees(190)),
	L1(Rotation2d.fromDegrees(190)),
	PRE_L2(Rotation2d.fromDegrees(-13)),
	L2(Rotation2d.fromDegrees(-13)),
	PRE_L3(Rotation2d.fromDegrees(10)),
	L3(Rotation2d.fromDegrees(10)),
	PRE_L4(Rotation2d.fromDegrees(55)),
	L4(Rotation2d.fromDegrees(-25)),
	POST_LOW_ALGAE_REMOVE(Rotation2d.fromDegrees(-30)),
	LOW_ALGAE_REMOVE(Rotation2d.fromDegrees(-30)),
	PRE_HIGH_ALGAE_REMOVE(Rotation2d.fromDegrees(-30)),
	HIGH_ALGAE_REMOVE(Rotation2d.fromDegrees(-30)),
	PRE_NET(Rotation2d.fromDegrees(60)),
	NET(Rotation2d.fromDegrees(45)),
	PROCESSOR_OUTTAKE(Rotation2d.fromDegrees(180)),
	CLIMB(Rotation2d.fromDegrees(-11));

	private final Rotation2d position;

	ArmState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
