package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.swerveconstants.RealSwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;

public class RealModuleConstants {

	private static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;

	private static final double COUPLING_RATIO = 0.59;

	protected static ModuleConstants getModuleConstants(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		return new ModuleConstants(
			modulePosition,
			swerveType.getLogPath(),
			WHEEL_DIAMETER_METERS,
			COUPLING_RATIO,
			RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND
		);
	}

}
