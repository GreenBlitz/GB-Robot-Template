package frc.robot.subsystems.swerve.swerveinterface;

import frc.robot.subsystems.swerve.ModuleUtils;
import frc.robot.subsystems.swerve.mk4iswerve.mk4imodules.MK4IModule;
import frc.robot.subsystems.swerve.replayswerve.ReplayModule;
import frc.robot.subsystems.swerve.simulationswerve.SimulationModule;
import frc.utils.RobotTypeUtils;

public class ModuleFactory {

    public static IModule createModule(ModuleUtils.ModuleName moduleName){
        return switch (RobotTypeUtils.getRobotType()){
            case REAL -> new MK4IModule(moduleName);
            case SIMULATION -> new SimulationModule(moduleName);
            case REPLAY -> new ReplayModule();
        };
    }

}
