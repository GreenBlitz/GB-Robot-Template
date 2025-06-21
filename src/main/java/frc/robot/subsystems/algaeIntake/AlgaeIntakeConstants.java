package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeIntakeConstants {

	public static final int NUMBER_OF_VALUES_IN_MEDIAN = 20;
	public static final int ALGAE_SENSOR_CHANNEL = 4;
	public static final Rotation2d MAXIMAL_PIVOT_VELOCITY_TO_UPDATE_FILTER_ANGLE_PER_SECOND = Rotation2d.fromDegrees(6);
	public static final double DISTANCE_FROM_SENSOR_TO_CONSIDER_ALGAE_IN_METERS = 0.05;

}
