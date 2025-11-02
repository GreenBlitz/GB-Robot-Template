package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Rotation2d;

public enum LimelightType {

	LIMELIGHT3(Rotation2d.fromDegrees(62.5), Rotation2d.fromDegrees(48.9)),
	LIMELIGHT3GB(Rotation2d.fromDegrees(110), Rotation2d.fromDegrees(70)), // temp value
	LIMELIGHT4(Rotation2d.fromDegrees(82.0), Rotation2d.fromDegrees(56.2));

	private final Rotation2d horizontalFieldOfView;
	private final Rotation2d verticalFieldOfViewY;

	LimelightType(Rotation2d horizontalFieldOfView, Rotation2d verticalFieldOfView) {
		this.horizontalFieldOfView = horizontalFieldOfView;
		this.verticalFieldOfViewY = verticalFieldOfView;
	}

	public Rotation2d getHorizontalFieldOfView() {
		return horizontalFieldOfView;
	}

	public Rotation2d getVerticalFieldOfView() {
		return verticalFieldOfViewY;
	}


}
