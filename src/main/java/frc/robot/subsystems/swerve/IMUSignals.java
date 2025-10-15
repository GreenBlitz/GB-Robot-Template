package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.*;
import frc.robot.hardware.interfaces.InputSignal;

public record IMUSignals(
	InputSignal<Rotation2d> pitchSignal,
	InputSignal<Rotation2d> rollSignal,
	InputSignal<Rotation2d> yawSignal,
	InputSignal<Rotation2d> rollAngularVelocitySignal,
	InputSignal<Rotation2d> pitchAngularVelocitySignal,
	InputSignal<Rotation2d> yawAngularVelocitySignal,
	InputSignal<Double> xAccelerationSignalG,
	InputSignal<Double> yAccelerationSignalG,
	InputSignal<Double> zAccelerationSignalG
) {

	public Rotation3d getAngularVelocity() {
		return new Rotation3d(
			this.rollAngularVelocitySignal().getLatestValue().getRadians(),
			this.pitchAngularVelocitySignal().getLatestValue().getRadians(),
			this.yawAngularVelocitySignal().getLatestValue().getRadians()
		);
	}

	public Rotation3d getOrientation() {
		return new Rotation3d(
			this.rollSignal().getLatestValue().getRadians(),
			this.pitchSignal().getLatestValue().getRadians(),
			this.yawSignal().getLatestValue().getRadians()
		);
	}

	public Translation3d getAccelerationG() {
		return new Translation3d(
			this.xAccelerationSignalG().getLatestValue(),
			this.yAccelerationSignalG().getLatestValue(),
			this.zAccelerationSignalG().getLatestValue()
		);
	}

}
