package frc.robot.subsystems.swerve.states;

public enum DriveSpeed {

	NORMAL(1, 1),
	SLOW(0.4, 0.4);

	private final double translationSpeedFactor;
	private final double rotationSpeedFactor;

	DriveSpeed(double translationSpeedFactor, double rotationSpeedFactor) {
		this.translationSpeedFactor = translationSpeedFactor;
		this.rotationSpeedFactor = rotationSpeedFactor;
	}

	public double getTranslationSpeedFactor() {
		return translationSpeedFactor;
	}

	public double getRotationSpeedFactor() {
		return rotationSpeedFactor;
	}

}
