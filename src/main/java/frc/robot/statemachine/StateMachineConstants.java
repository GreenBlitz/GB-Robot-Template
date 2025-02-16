package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Translation2d;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.5;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 1;
	public static final double CLOSE_SUPERSTRUCTURE_DISTANCE_FROM_BRANCH_METERS = 0.3;

	public static final Translation2d CLOSE_SUPERSTRUCTURE_LENGTH_AND_WIDTH = new Translation2d(0.3, 0.5);

	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;

}
