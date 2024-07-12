package frc.robot.subsystems.swerve.modules.factory;

import frc.robot.Robot;
import frc.robot.constants.DeviceIDs.ModulesIDs;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.mk4i.MK4IModule;
import frc.robot.subsystems.swerve.modules.replay.ReplayModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;

public class ModuleFactory {

    public static IModule createModule(ModuleUtils.ModuleName moduleName, double maxVelocityMetersPerSecond) {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> new MK4IModule(MK4IModuleContainer.mk4iModuleConstants(maxVelocityMetersPerSecond, ModulesIDs.moduleIDS[moduleName.getIndex()]));
            case SIMULATION -> new SimulationModule(moduleName, SimulationModuleContainer.simulationModuleConstants(maxVelocityMetersPerSecond));
            case REPLAY -> new ReplayModule();
        };
    }

}
