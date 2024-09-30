package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;

public enum FieldStartingPositions {

	// ! should have data from the actual field
	// TODO: add those
	NORTH_BLUE_ALLIANCE(new Pose2d()),
	SOUTH_BLUE_ALLIANCE(new Pose2d()),
	MIDDLE_BLUE_ALLIANCE(new Pose2d()),
	NORTH_RED_ALLIANCE(new Pose2d()),
	SOUTH_RED_ALLIANCE(new Pose2d()),
	MIDDLE_RED_ALLIANCE(new Pose2d());

	private final Pose2d startingPose;

	FieldStartingPositions(Pose2d startingPose) {
		this.startingPose = startingPose;
	}

	public Pose2d getStartingPose() {
		return this.startingPose;
	}

}
