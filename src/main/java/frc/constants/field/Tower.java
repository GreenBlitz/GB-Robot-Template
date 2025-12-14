package frc.constants.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Tower {

	CLOSE_TOWER(new Pose2d(10, 4, Rotation2d.kZero)),
	FAR_TOWER(new Pose2d(8, 4, Rotation2d.k180deg)),
	RIGHT_TOWER(new Pose2d(9, 3, Rotation2d.kCCW_90deg.times(-1))),
	LEFT_TOWER(new Pose2d(9, 5, Rotation2d.kCCW_90deg));

	private final Pose2d tower;

	Tower(Pose2d tower) {
		this.tower = tower;
	}

	public Pose2d getPose() {
		return tower;
	}

}
