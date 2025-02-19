package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.5;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 1;

	public static final Translation2d CLOSE_SUPERSTRUCTURE_LENGTH_AND_WIDTH = new Translation2d(0.79, 1.03);

	public static final double INTACK_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;

	public static final Rotation2d ARM_POSITION_TO_CLOSE_ELEVATOR_L4 = Rotation2d.fromDegrees(37);

}
