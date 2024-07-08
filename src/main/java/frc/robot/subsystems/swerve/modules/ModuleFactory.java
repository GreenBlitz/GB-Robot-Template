package frc.robot.subsystems.swerve.modules;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.modules.mk4i.MK4IModule;
import frc.robot.subsystems.swerve.modules.mk4i.MK4IModuleConstants;
import frc.robot.subsystems.swerve.modules.replay.ReplayModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;

public class ModuleFactory {

    public static IModule createModule(ModuleUtils.ModuleName moduleName) {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> new MK4IModule(MK4IModuleConstants.MODULE_CONFIG_OBJECTS[moduleName.getIndex()]);
            case SIMULATION -> new SimulationModule(moduleName);
            case REPLAY -> new ReplayModule();
        };
    }

}
