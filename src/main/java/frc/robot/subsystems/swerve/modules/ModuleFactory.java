package frc.robot.subsystems.swerve.modules;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.ModuleUtils;
import frc.robot.subsystems.swerve.modules.mk4imodule.MK4IModule;
import frc.robot.subsystems.swerve.modules.simulationmodule.SimulationModule;

public class ModuleFactory {

    public static IModule createModule(ModuleUtils.ModuleName moduleName) {
        return switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> new MK4IModule(moduleName);
            case SIMULATION -> new SimulationModule(moduleName);
            case REPLAY -> new ReplayModule();
        };
    }

}
