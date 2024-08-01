package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.Modules;
import frc.robot.subsystems.swerve.modules.replay.EmptyModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModule;

import static frc.robot.Robot.ROBOT_TYPE;

public class ModulesFactory {

    public static Modules create() {
        return switch (ROBOT_TYPE) {
            case REAL -> getModules(
                    new TalonFXModule[]{
                            new TalonFXModule(RealModulesConstants.MODULE_CONSTANTS[0]),
                            new TalonFXModule(RealModulesConstants.MODULE_CONSTANTS[1]),
                            new TalonFXModule(RealModulesConstants.MODULE_CONSTANTS[2]),
                            new TalonFXModule(RealModulesConstants.MODULE_CONSTANTS[3])
                    }
            );
            case SIMULATION -> new Modules(new Module[]{
               new Module(ModuleUtils.ModuleName.FRONT_LEFT, ),
               new Module(),
               new Module(),
               new Module()
            });
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
