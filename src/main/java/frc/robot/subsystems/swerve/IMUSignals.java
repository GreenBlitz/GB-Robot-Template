package frc.robot.subsystems.swerve;

import frc.utils.costomUnits.Angle2d;
import org.wpilib.math.geometry.*;
import frc.robot.hardware.interfaces.InputSignal;

public record IMUSignals(
	InputSignal<Angle2d> rollSignal,
	InputSignal<Angle2d> pitchSignal,
	InputSignal<Angle2d> yawSignal,
	InputSignal<Angle2d> rollAngularVelocitySignal,
	InputSignal<Angle2d> pitchAngularVelocitySignal,
	InputSignal<Angle2d> yawAngularVelocitySignal,
	InputSignal<Double> xAccelerationGSignal,
	InputSignal<Double> yAccelerationGSignal,
	InputSignal<Double> zAccelerationGSignal
) {

	public Angle2d[] getLatestAngularVelocity() {
		return new Angle2d[] {
			rollAngularVelocitySignal.getLatestValue(),
			pitchAngularVelocitySignal.getLatestValue(),
			yawAngularVelocitySignal.getLatestValue()};
	}

	public Rotation3d[] getAllOrientations() {
		Angle2d[] allRollValues = rollSignal.asArray();
		Angle2d[] allPitchValues = pitchSignal.asArray();
		Angle2d[] allYawValues = yawSignal.asArray();
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

	public Translation3d getLatestAccelerationGMetersPerSecondSquare() {
		return new Translation3d(
			xAccelerationGSignal().getLatestValue(),
			yAccelerationGSignal().getLatestValue(),
			zAccelerationGSignal().getLatestValue()
		);
	}

}
