package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;

public class ModuleConstantsFactory {

    public static ModuleConstants create(SwerveName swerveName, ModuleUtils.ModuleType moduleType) {
        return switch (moduleType) {
            case TALON_FX -> getTalonFXModuleConstants(swerveName);
        };
    }

    private static ModuleConstants getTalonFXModuleConstants(SwerveName swerveName) {
        return switch (Robot.ROBOT_TYPE) {
            case REAL, REPLAY -> RealModuleConstants.getModuleConstants(swerveName, ModuleUtils.ModuleType.TALON_FX);
            case SIMULATION -> SimulationModuleConstants.getModuleConstants(swerveName, ModuleUtils.ModuleType.TALON_FX);
        };
    }

}
