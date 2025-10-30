package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Translation2d;

public enum LimelightType {

	LIMELIGHT3(new Translation2d(62.5, 48.9)), //horizontal-vertical in degrees
	LIMELIGHT3GB(new Translation2d(72.25, 52.65)), //temp value
	LIMELIGHT4(new Translation2d(82, 56.2));

	private final Translation2d fieldOfView;

	LimelightType(Translation2d fieldOfViewY) {
		this.fieldOfView = fieldOfViewY;
	}

	public double getFieldOfViewX() {
		return fieldOfView.getX();
	}

	public double getFieldOfViewY() {
		return fieldOfView.getY();
	}


}
