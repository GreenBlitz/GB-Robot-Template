package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.swerve.ChassisPowers;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.59;
	public static final double DISTANCE_TO_BRANCH_FOR_STARTING_PATH = 1;
	public static final double NET_SCORING_OPEN_SUPERSTRUCTURE_X_POSITION_METERS = 6.5;
	public static final double SCORE_NET_X_POSITION_METERS = 7.63;
	public static final double MIN_NET_SCORING_Y_POSITION = 4.5;
	public static final double VELOCITY_BETWEEN_OPEN_SUPERSTRUCTURE_AND_SCORE_TO_NET_METERS_PER_SECOND = 1.5;
	public static final double PUSH_ALGAE_OUT_TIME_SECONDS = 0.075;

	public static final ChassisPowers SWERVE_POWERS_TO_PROCESSOR = new ChassisPowers();
	static {
		SWERVE_POWERS_TO_PROCESSOR.yPower = -0.1;
	}
	public static final double TIME_TO_RELEASE_ALGAE_TO_PROCESSOR = 1;
	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0;
	public static final double NET_OUTTAKE_TIME_SECONDS = 0.75;
	public static final double ALGAE_INTAKE_TIME_AFTER_SENSOR_SECONDS = 0;
	public static final double ALGAE_OUTTAKE_FROM_INTAKE_TIME_AFTER_SENSOR_SECONDS = 0.5;
	public static final double ALGAE_TRANSFER_TO_END_EFFECTOR_TIME_SECONDS = 0.4;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0;
	public static final Rotation2d ARM_POSITION_TO_DEPLOY_LIFTER = Rotation2d.fromDegrees(26 + ArmConstants.POSITION_OFFSET.getDegrees());
	public static final double ELEVATOR_POSITION_TO_CLOSE_CLIMB = 0.39;
	public static final double ELEVATOR_POSITION_FOR_OPENING = 0.3;
	public static final double ELEVATOR_POSITION_TO_MOVE_ARM_TO_SCORE_L4 = 0.5;


	public static final double MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND = 1;
	public static final double MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED = 2.5;
	public static final Rotation2d MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND = Rotation2d.fromRadians(4);
	public static final Rotation2d MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED = Rotation2d.fromRadians(4);

}
