package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.subsystems.elevator.ElevatorPhysicalProperties;
import frc.utils.math.StandardDeviations2D;

public class RobotConstants {

	public static final double DEFAULT_SIGNALS_FREQUENCY_HERTZ = 60;

	public static final String SUBSYSTEM_LOGPATH_PREFIX = "Subsystems";

	public static final ElevatorPhysicalProperties elevatorPhysicalProperties = new ElevatorPhysicalProperties(
		Rotation2d.fromRotations(0),
		Rotation2d.fromRotations(1),
		Units.Meter.of(1),
		Units.Meter.of(0)
	);
	public static final Distance elevatorStartingPosition = Units.Meters.of(0);
	public static final double elevatorRealKp = 2;
	public static final double elevatorSimKp = 2;
	public static final FeedbackConfigs elevatorFeedbackConfigs = new FeedbackConfigs();
	public static final Mass elevatorMass = Units.Grams.of(67);
	public static final Distance elevatorRadius = Units.Centimeter.of(67);

	public static final StandardDeviations2D DEFAULT_TAG_DISTANCE_FACTORS = new StandardDeviations2D(0.5);
	public static final StandardDeviations2D DEFAULT_STD_DEV_FACTORS = new StandardDeviations2D(0.15);
	public static final StandardDeviations2D DEFAULT_VISIBLE_TAGS_EXPONENTS = new StandardDeviations2D(0.4);
	public static final StandardDeviations2D DEFAULT_STD_DEV_ADDITIONS = new StandardDeviations2D(0.011);

	// data fetched from:
	// https://www.mt.com/mt_ext_files/Editorial/Generic/7/NewtonFactor_Editorial-Generic_1149155213028_files/Newton%20Factor%20and%20Values.pdf
	public static final double G = 9.7933;

}
