package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.arm.ArmConstants;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.52;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 0.8;
	public static final double DISTANCE_TO_BRANCH_FOR_STARTING_PATH = 1;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_NET_METERS = 2.4;
	public static final double SCORE_DISTANCE_FROM_NET_METERS = 2.1;
	public static final double POWER_FOR_MANUAL_CLIMB = -0.5;

	public static final double DISTANCE_FROM_CORAL_STATION_SLOT_TO_START_AIM_ASSIST_METERS = 2.2;

	// Field relative
	public static final Translation2d CLOSE_SUPERSTRUCTURE_LENGTH_AND_WIDTH = new Translation2d(0.6, 1.03);
	public static final Translation2d SCORE_DISTANCES_FROM_MIDDLE_OF_BARGE_METRES = new Translation2d(2.8, 0.1);

	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0;
	public static final double NET_OUTTAKE_TIME_AFTER_LIMIT_SWITCH_SECONDS = 0;
	public static final double ALGAE_REMOVE_TIME_AFTER_LIMIT_SWITCH_SECONDS = 0;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0;
	public static final Rotation2d ARM_POSITION_TO_CLOSE_ELEVATOR_L4 = Rotation2d.fromDegrees(63 + ArmConstants.POSITION_OFFSET.getDegrees());
	public static final Rotation2d ARM_POSITION_TO_DEPLOY_LIFTER = Rotation2d.fromDegrees(26 + ArmConstants.POSITION_OFFSET.getDegrees());
	public static final Rotation2d ARM_POSITION_TO_RELEASE_NET = Rotation2d.fromDegrees(175 + ArmConstants.POSITION_OFFSET.getDegrees());
	public static final double ELEVATOR_POSITION_TO_CLOSE_CLIMB = 0.39;
	public static final double ELEVATOR_POSITION_FOR_OPENING = 0.39;
	public static final double ELEVATOR_POSITION_TO_MOVE_ARM_TO_SCORE_L4 = 0.5;
	public static final double ELEVATOR_POSITION_TO_CLOSE_ARM = 0.4;
	public static final double ELEVATOR_POSITION_TO_RELEASE_NET = 0.4;

	public static final double MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND = 0.75;
	public static final double MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED = 1.25;
	public static final Rotation2d MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND = Rotation2d.fromRadians(4);
	public static final Rotation2d MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED = Rotation2d.fromRadians(3);

}
