package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Rotation2d;

public enum LimelightVersion {

	LIMELIGHT_3(Rotation2d.fromDegrees(62.5), Rotation2d.fromDegrees(48.9)),
	LIMELIGHT_3G(Rotation2d.fromDegrees(82), Rotation2d.fromDegrees(56.2)),
	LIMELIGHT_4(Rotation2d.fromDegrees(82), Rotation2d.fromDegrees(56.2));

	private final Rotation2d horizontalFieldOfView;
	private final Rotation2d verticalFieldOfView;

	LimelightVersion(Rotation2d horizontalFieldOfView, Rotation2d verticalFieldOfView) {
		this.horizontalFieldOfView = horizontalFieldOfView;
		this.verticalFieldOfView = verticalFieldOfView;
	}

	public Rotation2d getHorizontalFieldOfView() {
		return horizontalFieldOfView;
	}

	public Rotation2d getVerticalFieldOfView() {
		return verticalFieldOfView;
	}

}
