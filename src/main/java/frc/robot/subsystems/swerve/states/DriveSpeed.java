package frc.robot.subsystems.swerve.states;

public enum DriveSpeed {

	NORMAL(1, 1),
	SLOW(0.5, 0.5);

	public final double translationSpeedFactor;
	public final double rotationSpeedFactor;

	DriveSpeed(double translationSpeedFactor, double rotationSpeedFactor) {
		this.translationSpeedFactor = translationSpeedFactor;
		this.rotationSpeedFactor = rotationSpeedFactor;
	}

}
