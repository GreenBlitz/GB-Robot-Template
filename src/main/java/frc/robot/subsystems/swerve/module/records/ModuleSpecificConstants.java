package frc.robot.subsystems.swerve.module.records;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.utils.Conversions;

public record ModuleSpecificConstants(
	String logPath,
	double wheelDiameterMeters,
	double couplingRatio,
	Rotation2d velocityAt12VoltsPerSecond,
	Translation2d positionFromCenterMeters
) {

	public ModuleSpecificConstants(
		ModuleUtil.ModulePosition modulePosition,
		String logPathPrefix,
		double wheelDiameterMeters,
		double couplingRatio,
		double velocityAt12VoltsMetersPerSecond,
		Translation2d positionFromCenterMeters
	) {
		this(
			logPathPrefix + ModuleConstants.MODULES_LOG_PATH_ADDITION + "/" + modulePosition,
			wheelDiameterMeters,
			couplingRatio,
			Conversions.distanceToAngle(velocityAt12VoltsMetersPerSecond, wheelDiameterMeters),
			positionFromCenterMeters
		);
	}

}
