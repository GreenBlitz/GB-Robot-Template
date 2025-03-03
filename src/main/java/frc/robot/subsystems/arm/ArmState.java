package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorState;

public enum ArmState {

	STAY_IN_PLACE(Rotation2d.fromDegrees(Double.NaN), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
	CLOSED(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	FIRST_WAYPOINT_TO_CLOSE(Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	SECOND_WAYPOINT_TO_CLOSE(Rotation2d.fromDegrees(110), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	START_GAME(Rotation2d.fromDegrees(207), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	MID_WAY_CLOSE(Rotation2d.fromDegrees(130), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	INTAKE(Rotation2d.fromDegrees(186), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	ALGAE_OUTTAKE(Rotation2d.fromDegrees(185), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	PRE_L1(Rotation2d.fromDegrees(190), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	L1(Rotation2d.fromDegrees(190), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	PRE_L2(Rotation2d.fromDegrees(-13), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	L2(Rotation2d.fromDegrees(-13), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	PRE_L3(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	L3(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	PRE_L4(Rotation2d.fromDegrees(55), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	L4(Rotation2d.fromDegrees(-25), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	LOW_ALGAE_REMOVE(Rotation2d.fromDegrees(-30), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	HIGH_ALGAE_REMOVE(Rotation2d.fromDegrees(-30), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	PRE_NET(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	NET(Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	PROCESSOR_OUTTAKE(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100)),
	CLIMB(Rotation2d.fromDegrees(-11), Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(100));

	private final Rotation2d position;
	private final Rotation2d maxVelocityRotation2dPerSecond;
	private final Rotation2d maxAccelerationRotation2dPerSecondSquared;

	ArmState(Rotation2d position, Rotation2d maxVelocityRotation2dPerSecond, Rotation2d maxAccelerationRotation2dPerSecondSquared) {
		this.position = position;
		this.maxVelocityRotation2dPerSecond = maxVelocityRotation2dPerSecond;
		this.maxAccelerationRotation2dPerSecondSquared = maxAccelerationRotation2dPerSecondSquared;
	}

	public Rotation2d getPosition() {
		return position;
	}

	public Rotation2d getMaxVelocityRotation2dPerSecond() {
		return maxVelocityRotation2dPerSecond;
	}

	public Rotation2d getMaxAccelerationRotation2dPerSecondSquared() {
		return maxAccelerationRotation2dPerSecondSquared;
	}

}
