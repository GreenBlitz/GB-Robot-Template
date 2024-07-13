package frc.robot.subsystems.swerve.modules.factory;

import frc.robot.Robot;
import frc.robot.constants.DeviceIDs.ModulesIDs;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModule;
import frc.robot.subsystems.swerve.modules.replay.ReplayModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;

public class ModuleFactory {

    public static IModule createModule(ModuleUtils.ModuleName moduleName, double velocityAt12VoltsMetersPerSecond) {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> new TalonFXModule(MK4IModuleContainer.mk4iModuleConstants(velocityAt12VoltsMetersPerSecond, ModulesIDs.MODULE_IDS[moduleName.getIndex()]));
            case SIMULATION -> new SimulationModule(moduleName, SimulationModuleContainer.simulationModuleConstants(velocityAt12VoltsMetersPerSecond));
            case REPLAY -> new ReplayModule();
        };
    }

}
