package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;

public class ModuleConstantsFactory {

    public static ModuleConstants create(SwerveName swerveName, ModuleUtils.ModuleType moduleType){
        return switch (Robot.ROBOT_TYPE){
            case REAL, REPLAY -> getRealModuleConstants(swerveName, moduleType);
            case SIMULATION -> SimulationModuleConstants.getModuleConstants(swerveName, moduleType);
        };
    }

    private static ModuleConstants getRealModuleConstants(SwerveName swerveName, ModuleUtils.ModuleType moduleType) {
        return switch (moduleType) {
            case TALON_FX -> RealModuleConstants.getModuleConstants(swerveName, moduleType);
        };
    }

}
