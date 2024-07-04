package frc.robot.subsystems.swerve.modules.moduleinterface;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.mk4i.MK4IModule;
import frc.robot.subsystems.swerve.modules.replay.ReplayModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;

public class ModuleFactory {

    public static IModule createModule(ModuleUtils.ModuleName moduleName) {
        return switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> new MK4IModule(moduleName);
            case SIMULATION -> new SimulationModule(moduleName);
            case REPLAY -> new ReplayModule();
        };
    }

}
