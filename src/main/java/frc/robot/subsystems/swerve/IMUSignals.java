package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.*;
import frc.robot.hardware.interfaces.InputSignal;

public record IMUSignals(
	InputSignal<Rotation2d> rollSignal,
	InputSignal<Rotation2d> pitchSignal,
	InputSignal<Rotation2d> yawSignal,
	InputSignal<Rotation2d> rollAngularVelocitySignal,
	InputSignal<Rotation2d> pitchAngularVelocitySignal,
	InputSignal<Rotation2d> yawAngularVelocitySignal,
	InputSignal<Double> xAccelerationGSignal,
	InputSignal<Double> yAccelerationGSignal,
	InputSignal<Double> zAccelerationGSignal
) {

	public Rotation2d[] getLatestAngularVelocity() {
		return new Rotation2d[] {
			rollAngularVelocitySignal.getLatestValue(),
			pitchAngularVelocitySignal.getLatestValue(),
			yawAngularVelocitySignal.getLatestValue()};
	}

	public Rotation3d[] getAllOrientations() {
		Rotation2d[] allRollValues = rollSignal.asArray();
		Rotation2d[] allPitchValues = pitchSignal.asArray();
		Rotation2d[] allYawValues = yawSignal.asArray();
		Rotation3d[] allOrientations = new Rotation3d[Math.min(Math.min(allRollValues.length, allPitchValues.length), allYawValues.length)];

		for (int i = 0; i < allOrientations.length; i++) {
			allOrientations[i] = new Rotation3d(allRollValues[i].getRadians(), allPitchValues[i].getRadians(), allYawValues[i].getRadians());
		}
		return allOrientations;
	}

	public Rotation3d getLatestOrientation() {
		return new Rotation3d(
			rollSignal.getLatestValue().getRadians(),
			pitchSignal.getLatestValue().getRadians(),
			yawSignal.getLatestValue().getRadians()
		);
	}

	public Translation3d[] getAllAccelerationsG() {
		Double[] allXAccelerations = xAccelerationGSignal.asArray();
		Double[] allYAccelerations = yAccelerationGSignal.asArray();
		Double[] allZAccelerations = zAccelerationGSignal.asArray();
		Translation3d[] allAccelerations = new Translation3d[Math
			.min(Math.min(allXAccelerations.length, allYAccelerations.length), allZAccelerations.length)];

		for (int i = 0; i < allAccelerations.length; i++) {
			allAccelerations[i] = new Translation3d(allXAccelerations[i], allYAccelerations[i], allZAccelerations[i]);
		}
		return allAccelerations;
	}

	public Translation3d getLatestAccelerationG() {
		return new Translation3d(
			xAccelerationGSignal().getLatestValue(),
			yAccelerationGSignal().getLatestValue(),
			zAccelerationGSignal().getLatestValue()
		);
	}

}
