package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ModuleState {

	IDLE(null, Rotation2d.fromDegrees(0));

	private final Rotation2d driveVelocityRotation2dPerSecond;
	private final Rotation2d angle;

	ModuleState(Rotation2d angle, Rotation2d driveVelocityRotation2dPerSecond) {
		this.angle = angle;
		this.driveVelocityRotation2dPerSecond = driveVelocityRotation2dPerSecond;
	}

	public Rotation2d getDriveVelocityRotation2dPerSecond() {
		return driveVelocityRotation2dPerSecond;
	}

	public Rotation2d getAngle() {
		return angle;
	}

}
