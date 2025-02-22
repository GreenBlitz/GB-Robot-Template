package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.56;
	public static final double ROBOT_ALGAE_DISTANCE_FROM_REEF_METERS = 0.53;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 0.7;
	public static final double DISTANCE_TO_BRANCH_FOR_STARTING_PATH = 0.7;

	public static final Translation2d CLOSE_SUPERSTRUCTURE_LENGTH_AND_WIDTH = new Translation2d(0.79, 1.03);

	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0;
	public static final double ALGAE_REMOVE_TIME_AFTER_LIMIT_SWITCH_SECONDS = 0;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0;

	public static final Rotation2d ARM_POSITION_TO_CLOSE_ELEVATOR_L4 = Rotation2d.fromDegrees(37);
	public static final double ELEVATOR_POSITION_TO_MOVE_ARM_TO_SCORE_L4 = 0.5;
	public static final double ELEVATOR_POSITION_TO_CLOSE_ARM = 0.2;

	public static final double MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND = 0.5;
	public static final double MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED = 0.5;
	public static final Rotation2d MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND = Rotation2d.fromRadians(1);
	public static final Rotation2d MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED = Rotation2d.fromRadians(0.5);

}
