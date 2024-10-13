package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;
import frc.utils.battery.BatteryUtils;

public record ModuleConstants(String logPath, double wheelDiameterMeters, double couplingRatio, Rotation2d velocityAt12VoltsPerSecond) {

	public ModuleConstants(
		ModuleUtils.ModulePosition modulePosition,
		String logPathPrefix,
		double wheelDiameterMeters,
		double couplingRatio,
		double velocityAt12VoltsMetersPerSecond
	) {
		this(
			logPathPrefix + LOG_PATH_ADDITION + modulePosition + "/",
			wheelDiameterMeters,
			couplingRatio,
			Conversions.distanceToAngle(velocityAt12VoltsMetersPerSecond, wheelDiameterMeters)
		);
	}

	public static final double VOLTAGE_COMPENSATION_SATURATION = BatteryUtils.DEFAULT_VOLTAGE;
	public static final boolean DEFAULT_IS_CLOSE_LOOP = true;
	public static final String LOG_PATH_ADDITION = "Modules/";

}
