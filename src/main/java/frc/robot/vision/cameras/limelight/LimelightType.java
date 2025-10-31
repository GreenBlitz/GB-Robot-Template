package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Rotation2d;

public enum LimelightType {

	LIMELIGHT3(Rotation2d.fromDegrees(62.5), Rotation2d.fromDegrees(48.9)),
	LIMELIGHT3GB(Rotation2d.fromDegrees(72.25),Rotation2d.fromDegrees(52.65)), // temp value
	LIMELIGHT4(Rotation2d.fromDegrees(82.0),  Rotation2d.fromDegrees(56.2));

	private final Rotation2d horizontalFieldOfView;
	private final Rotation2d verticalFieldOfView;

	LimelightType(Rotation2d horizontalFieldOfView, Rotation2d verticalFieldOfView) {
		this.horizontalFieldOfView = horizontalFieldOfView;
		this.verticalFieldOfView = verticalFieldOfView;
	}

	public Rotation2d getFieldOfViewX() {
		return horizontalFieldOfView;
	}

	public Rotation2d getFieldOfViewY() {
		return verticalFieldOfView;
	}


}
