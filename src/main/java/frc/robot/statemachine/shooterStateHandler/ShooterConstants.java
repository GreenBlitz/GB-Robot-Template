package frc.robot.statemachine.shooterStateHandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.utils.InterpolationMap;
import frc.utils.LoggedNetworkRotation2d;

import java.util.Map;

public class ShooterConstants {

	public static final Rotation2d DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND = Rotation2d.fromRotations(40.0);
	public static final String LOG_PATH = "StateHandlers/ShooterStateHandler";

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

	public static final LoggedNetworkRotation2d turretCalibrationAngle = new LoggedNetworkRotation2d("TurretAngle", Rotation2d.fromDegrees(0.0));
	public static final LoggedNetworkRotation2d hoodCalibrationAngle = new LoggedNetworkRotation2d("HoodAngle", Rotation2d.fromDegrees(0.0));
	public static final LoggedNetworkRotation2d flywheelCalibrationRotations = new LoggedNetworkRotation2d(
		"FlywheelRotations",
		Rotation2d.fromRotations(0.0)
	);
	public static final Rotation2d MAX_DISTANCE_FROM_SCREW_NOT_TO_ROTATE = Rotation2d.fromDegrees(15);

}
