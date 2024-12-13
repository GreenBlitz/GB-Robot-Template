package frc.robot.subsystems.swerve.module.records;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.utils.Conversions;

public record ModuleSpecificConstants(String logPath, double wheelDiameterMeters, double couplingRatio, Rotation2d velocityAt12VoltsPerSecond) {

	public ModuleSpecificConstants(
		ModuleUtils.ModulePosition modulePosition,
		String logPathPrefix,
		double wheelDiameterMeters,
		double couplingRatio,
		double velocityAt12VoltsMetersPerSecond
	) {
		this(
			logPathPrefix + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/",
			wheelDiameterMeters,
			couplingRatio,
			Conversions.distanceToAngle(velocityAt12VoltsMetersPerSecond, wheelDiameterMeters)
		);
	}

}
