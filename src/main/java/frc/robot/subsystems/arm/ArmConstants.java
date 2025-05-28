package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.utils.InterpolationMap;

import java.util.Map;

public class ArmConstants {

	public static final Rotation2d POSITION_OFFSET = Rotation2d.fromDegrees(-16); // was 26

	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(231 + POSITION_OFFSET.getDegrees());
	public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(246 + POSITION_OFFSET.getDegrees());
	public static final Rotation2d ELEVATOR_CLOSED_REVERSED_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-8 + POSITION_OFFSET.getDegrees());
	public static final Rotation2d ELEVATOR_OPEN_REVERSED_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-24 + POSITION_OFFSET.getDegrees());
	public static final double ELEVATOR_HEIGHT_METERS_TO_CHANGE_SOFT_LIMIT = 0.3;
	public static final double LENGTH_METERS = 0.3;
	public static final double MASS_KG = 5;

	public static final Rotation2d CRUISE_VELOCITY_ANGLES_PER_SECOND = Rotation2d.fromRotations(3);
	public static final Rotation2d ACCELERATION_ANGLES_PER_SECOND_SQUARED = Rotation2d.fromRotations(3);

	public static final double CALIBRATION_MAX_POWER = 0.2;

	public static final InterpolationMap<Double, Rotation2d> L4_DISTANCE_ANGLE_MAP = new InterpolationMap<>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			0.0,
			Rotation2d.fromDegrees(0),
			0.48,
			Rotation2d.fromDegrees(0),
			0.59,
			Rotation2d.fromDegrees(7.5),
			0.65,
			Rotation2d.fromDegrees(10),
			0.70,
			Rotation2d.fromDegrees(18.5)
		)
	);

	public static final InterpolationMap<Double, Rotation2d> L3_DISTANCE_ANGLE_MAP = new InterpolationMap<>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			0.0,
			Rotation2d.fromDegrees(0),
			0.48,
			Rotation2d.fromDegrees(0),
			0.59,
			Rotation2d.fromDegrees(1.75),
			0.65,
			Rotation2d.fromDegrees(4.5),
			0.70,
			Rotation2d.fromDegrees(7)
		)
	);

	public static final InterpolationMap<Double, Rotation2d> L2_DISTANCE_ANGLE_MAP = new InterpolationMap<>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			0.0,
			Rotation2d.fromDegrees(0),
			0.48,
			Rotation2d.fromDegrees(0),
			0.59,
			Rotation2d.fromDegrees(3.75),
			0.65,
			Rotation2d.fromDegrees(6.5),
			0.70,
			Rotation2d.fromDegrees(9.5)
		)
	);

}
