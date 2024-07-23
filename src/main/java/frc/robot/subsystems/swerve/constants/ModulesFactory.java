package frc.robot.subsystems.swerve.constants;

import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.replay.ReplayModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModuleConstants;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModule;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModuleConstants;

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

    public static Module[] createModules(ModuleConstants[] moduleConstants) {
        return switch (ROBOT_TYPE) {
            case REAL -> getModules(
                    new TalonFXModule[]{
                            new TalonFXModule((TalonFXModuleConstants) moduleConstants[0]),
                            new TalonFXModule((TalonFXModuleConstants) moduleConstants[1]),
                            new TalonFXModule((TalonFXModuleConstants) moduleConstants[2]),
                            new TalonFXModule((TalonFXModuleConstants) moduleConstants[3])
                    }
            );
            case SIMULATION -> getModules(
                    new SimulationModule[]{
                            new SimulationModule(ModuleUtils.ModuleName.FRONT_LEFT, (SimulationModuleConstants) moduleConstants[0]),
                            new SimulationModule(ModuleUtils.ModuleName.FRONT_RIGHT, (SimulationModuleConstants) moduleConstants[1]),
                            new SimulationModule(ModuleUtils.ModuleName.BACK_LEFT, (SimulationModuleConstants) moduleConstants[2]),
                            new SimulationModule(ModuleUtils.ModuleName.BACK_RIGHT, (SimulationModuleConstants) moduleConstants[3])
                    }
            );
            case REPLAY -> getModules(
                    new ReplayModule[]{
                            new ReplayModule(),
                            new ReplayModule(),
                            new ReplayModule(),
                            new ReplayModule()
                    }
            );
        };
    }

}
