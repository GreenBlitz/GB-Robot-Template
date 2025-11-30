package frc.robot.statemachine.shooterStateHandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.utils.InterpolationMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Map;

public class ShooterConstants {

	public static final Rotation2d DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND = Rotation2d.fromRotations(40.0);
	public static final String LOG_PATH = "ShooterStateHandler";

	public static final InterpolationMap<Double, Rotation2d> HOOD_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(2.0, Rotation2d.fromDegrees(5.0))
	);

	public static final InterpolationMap<Double, Rotation2d> FLYWHEEL_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(2.0, Rotation2d.fromDegrees(5.0))
	);

	public static final LoggedNetworkNumber turretCalibrationVoltage = new LoggedNetworkNumber("TurretPower", 0);
	public static final LoggedNetworkNumber hoodCalibrationVoltage = new LoggedNetworkNumber("HoodPower", 0);
	public static final LoggedNetworkNumber flywheelCalibrationVoltage = new LoggedNetworkNumber("FlywheelPower", 0);

	public static final Rotation2d SCREW_LOCATION = Rotation2d.fromDegrees(0);
	public static final Rotation2d LENGTH_NOT_TO_TURN = Rotation2d.fromDegrees(30);

}
