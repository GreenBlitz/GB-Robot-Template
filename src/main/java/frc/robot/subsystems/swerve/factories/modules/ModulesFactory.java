package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.replay.EmptyModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModule;

import static frc.robot.Robot.ROBOT_TYPE;

public class ModulesFactory {

    private static Module[] getModules(IModule[] iModules) {
        return new Module[]{
                new Module(ModuleUtils.ModuleName.FRONT_LEFT, iModules[0]),
                new Module(ModuleUtils.ModuleName.FRONT_RIGHT, iModules[1]),
                new Module(ModuleUtils.ModuleName.BACK_LEFT, iModules[2]),
                new Module(ModuleUtils.ModuleName.BACK_RIGHT, iModules[3])
        };
    }

    public static Module[] create() {
        return switch (ROBOT_TYPE) {
            case REAL -> getModules(
                    new TalonFXModule[]{
                            new TalonFXModule(RealModulesConstants.MODULE_CONSTANTS[0]),
                            new TalonFXModule(RealModulesConstants.MODULE_CONSTANTS[1]),
                            new TalonFXModule(RealModulesConstants.MODULE_CONSTANTS[2]),
                            new TalonFXModule(RealModulesConstants.MODULE_CONSTANTS[3])
                    }
            );
            case SIMULATION -> getModules(
                    new SimulationModule[]{
                            new SimulationModule(ModuleUtils.ModuleName.FRONT_LEFT, SimulationModulesConstants.getModuleConstants()),
                            new SimulationModule(ModuleUtils.ModuleName.FRONT_RIGHT, SimulationModulesConstants.getModuleConstants()),
                            new SimulationModule(ModuleUtils.ModuleName.BACK_LEFT, SimulationModulesConstants.getModuleConstants()),
                            new SimulationModule(ModuleUtils.ModuleName.BACK_RIGHT, SimulationModulesConstants.getModuleConstants())
                    }

            );
            case REPLAY -> getModules(
                    new EmptyModule[]{
                            new EmptyModule(),
                            new EmptyModule(),
                            new EmptyModule(),
                            new EmptyModule()
                    }
            );
        };
    }

}
