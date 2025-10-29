package frc.robot.vision.cameras.limelight;

public enum FOV {

	LIMELIGHT3(0, 0),
	LIMELIGHT3GB(0, 0),
	LIMELIGHT4(0, 0);

	private double fieldOfViewX;
	private double fieldOfViewY;

	FOV(double fieldOfViewX, double fieldOfViewY) {
		this.fieldOfViewX = fieldOfViewX;
		this.fieldOfViewY = fieldOfViewY;
	}

	public double getFieldOfViewX() {
		return fieldOfViewX;
	}

	public double getFieldOfViewY() {
		return fieldOfViewY;
	}

}
