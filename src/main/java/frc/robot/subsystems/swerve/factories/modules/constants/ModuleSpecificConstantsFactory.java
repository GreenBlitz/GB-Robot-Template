package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.ModuleSpecificConstants;

public class ModuleSpecificConstantsFactory {

	private static ModuleSpecificConstants createSwerveModuleSpecificConstants(
		SwerveType swerveType,
		ModuleUtils.ModulePosition modulePosition
	) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealModuleConstants.getModuleSpecificConstants(swerveType, modulePosition);
			case SIMULATION -> SimulationModuleConstants.getModuleSpecificConstants(swerveType, modulePosition);
		};
	}

	public static ModuleSpecificConstants create(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveType) {
			case SWERVE -> createSwerveModuleSpecificConstants(swerveType, modulePosition);
		};
	}

}
