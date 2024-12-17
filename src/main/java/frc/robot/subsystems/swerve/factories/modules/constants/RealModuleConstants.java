package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.swerveconstants.RealSwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.ModuleSpecificConstants;

public class RealModuleConstants {

	private static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;
	private static final double COUPLING_RATIO = 0.59;

	protected static ModuleSpecificConstants getModuleSpecificConstants(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		return new ModuleSpecificConstants(
			modulePosition,
			swerveType.getLogPath(),
			WHEEL_DIAMETER_METERS,
			COUPLING_RATIO,
			RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND
		);
	}

}
