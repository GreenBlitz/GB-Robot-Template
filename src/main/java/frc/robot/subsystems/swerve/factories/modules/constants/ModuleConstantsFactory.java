package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;

public class ModuleConstantsFactory {

	public static ModuleConstants create(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveName) {
			case SWERVE -> createSwerveModuleConstants(swerveName, modulePosition);
		};
	}

	private static ModuleConstants createSwerveModuleConstants(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealModuleConstants.getModuleConstants(swerveName, modulePosition);
			case SIMULATION -> SimulationModuleConstants.getModuleConstants(swerveName, modulePosition);
		};
	}

}
