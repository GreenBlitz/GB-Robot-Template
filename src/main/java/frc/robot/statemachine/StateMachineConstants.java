package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.3;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 1;
	public static final double MIDDLE_OF_AIM_ASSIST_ACTIVATING_RECTANGLE_DISTANCE_FROM_SCORING_POSITION = 1;

	public static final Pose2d REEF_AIM_ASSIST_ACTIVATING_DISTANCES_FROM_CENTER_OF_AIM_ASSIST_RECTANGLE = new Pose2d(1, 2, new Rotation2d());

}
