package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.subsystems.swerve.SwerveType;
<<<<<<< HEAD
import frc.robot.subsystems.swerve.factories.swerveconstants.RealSwerveConstants;
=======
import frc.robot.subsystems.swerve.factories.swerveconstants.SimulationSwerveConstants;
>>>>>>> core-swerve
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;

public class SimulationModuleConstants {

	private static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;
	private static final double COUPLING_RATIO = 0;

	protected static ModuleConstants getModuleConstants(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		return new ModuleConstants(
			modulePosition,
			swerveType.getLogPath(),
			WHEEL_DIAMETER_METERS,
			COUPLING_RATIO,
<<<<<<< HEAD
			RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND
=======
			SimulationSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND
>>>>>>> core-swerve
		);
	}

}
