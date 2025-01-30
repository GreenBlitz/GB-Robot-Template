package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.records.ModuleSpecificConstants;

public class ModuleSpecificConstantsFactory {

	public static ModuleSpecificConstants create(String logPath, ModuleUtil.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
//            case REAL -> RealModuleConstants.getModuleSpecificConstants(logPath, modulePosition); TODO
			case SIMULATION, REAL -> SimulationModuleConstants.getModuleSpecificConstants(logPath, modulePosition);
		};
	}

}
