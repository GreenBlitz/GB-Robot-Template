package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;

public class ModuleConstantsFactory {

	private static ModuleConstants createSwerveModuleConstants(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealModuleConstants.getModuleConstants(swerveType, modulePosition);
			case SIMULATION -> null;// TODO
		};
	}

	public static ModuleConstants create(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveType) {
			case SWERVE -> createSwerveModuleConstants(swerveType, modulePosition);
		};
	}

}
