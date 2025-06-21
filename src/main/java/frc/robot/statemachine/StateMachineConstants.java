package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.swerve.ChassisPowers;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.59;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 0.8;
	public static final double DISTANCE_TO_BRANCH_FOR_STARTING_PATH = 1;
	public static final double NET_SCORING_OPEN_SUPERSTRUCTURE_X_POSITION_METERS = 7;
	public static final double SCORE_NET_X_POSITION_METERS = 7.73;
	public static final double MIN_NET_SCORING_Y_POSITION = 4.5;
	public static final double POWER_FOR_MANUAL_CLIMB = -0.5;

	public static final double DISTANCE_FROM_CORAL_STATION_SLOT_TO_START_AIM_ASSIST_METERS = 2.2;

	// Field relative
	public static final Translation2d CLOSE_SUPERSTRUCTURE_LENGTH_AND_WIDTH = new Translation2d(0.6, 1.03);
	public static final Translation2d SCORE_DISTANCES_FROM_MIDDLE_OF_BARGE_METRES = new Translation2d(1.6, 0.1);

	public static final ChassisPowers SWERVE_POWERS_TO_PROCESSOR = new ChassisPowers();
	static {
		SWERVE_POWERS_TO_PROCESSOR.yPower = -0.1;
	}
	public static final double TIME_TO_RELEASE_ALGAE_TO_PROCESSOR = 1;
	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0;
	public static final double NET_OUTTAKE_TIME_SECONDS = 1;
	public static final double ALGAE_REMOVE_TIME_AFTER_LIMIT_SWITCH_SECONDS = 0;
	public static final double ALGAE_INTAKE_TIME_AFTER_SENSOR_SECONDS = 0;
	public static final double ALGAE_INTAKE_MOVE_TO_OUTTAKE_POSITION_TIME_SECONDS = 0.8;
	public static final double ALGAE_OUTTAKE_FROM_INTAKE_TIME_AFTER_SENSOR_SECONDS = 0.2;
	public static final double ALGAE_TRANSFER_TO_END_EFFECTOR_TIME_SECONDS = 0.8;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0;
	public static final Rotation2d ARM_POSITION_TO_CLOSE_ELEVATOR_L4 = Rotation2d.fromDegrees(63 + ArmConstants.POSITION_OFFSET.getDegrees());
	public static final Rotation2d ARM_POSITION_TO_DEPLOY_LIFTER = Rotation2d.fromDegrees(26 + ArmConstants.POSITION_OFFSET.getDegrees());
	public static final Rotation2d ARM_POSITION_TO_RELEASE_NET = Rotation2d.fromDegrees(175 + ArmConstants.POSITION_OFFSET.getDegrees());
	public static final Rotation2d PIVOT_POSITION_TO_ALLOW_TRANSFER = Rotation2d.fromDegrees(45);
	public static final double ELEVATOR_POSITION_TO_CLOSE_CLIMB = 0.39;
	public static final double ELEVATOR_POSITION_FOR_OPENING = 0.3;
	public static final double ELEVATOR_POSITION_TO_MOVE_ARM_TO_SCORE_L4 = 0.5;
	public static final double ELEVATOR_POSITION_TO_CLOSE_ARM = 0.4;
	public static final double ELEVATOR_POSITION_TO_RELEASE_NET = 0.37;
	public static final double ELEVATOR_POSITION_TO_START_THROW_NET = 0.8;


	public static final double MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND = 1;
	public static final double MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED = 2.5;
	public static final Rotation2d MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND = Rotation2d.fromRadians(4);
	public static final Rotation2d MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED = Rotation2d.fromRadians(4);

}
