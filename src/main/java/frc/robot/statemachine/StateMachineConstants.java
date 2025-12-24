package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;

public class StateMachineConstants {

	public static final Rotation2d HEADING_TOLERANCE = Rotation2d.fromDegrees(10);
	public static final Rotation2d MAX_ANGLE_FROM_GOAL_CENTER = Rotation2d.fromDegrees(100);
	public static final double MAX_DISTANCE_TO_SHOOT_METERS = 500;
	public static final double SECONDS_TO_WAIT_AFTER_SHOOT = 0.2;

}
