package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.*;
import frc.robot.hardware.interfaces.InputSignal;

public record IMUSignals(
	InputSignal<Rotation2d> pitchSignal,
	InputSignal<Rotation2d> rollSignal,
	InputSignal<Rotation2d> yawSignal,
	InputSignal<Rotation2d> angularVelocityRollSignal,
	InputSignal<Rotation2d> angularVelocityPitchSignal,
	InputSignal<Rotation2d> angularVelocityYawSignal,
	InputSignal<Double> accelerationXSignal,
	InputSignal<Double> accelerationYSignal,
	InputSignal<Double> accelerationZSignal
) {

	public Rotation3d getMeasuredAngularVelocity() {
		return new Rotation3d(
			this.angularVelocityRollSignal().getLatestValue().getRadians(),
			this.angularVelocityPitchSignal().getLatestValue().getRadians(),
			this.angularVelocityYawSignal().getLatestValue().getRadians()
		);
	}

	public Rotation3d getMeasuredOrientation() {
		return new Rotation3d(
			this.rollSignal().getLatestValue().getRadians(),
			this.pitchSignal().getLatestValue().getRadians(),
			this.yawSignal().getLatestValue().getRadians()
		);
	}

	public Translation3d getMeasuredAcceleration() {
		return new Translation3d(
			this.accelerationXSignal().getLatestValue(),
			this.accelerationYSignal().getLatestValue(),
			this.accelerationZSignal().getLatestValue()
		);
	}

}
